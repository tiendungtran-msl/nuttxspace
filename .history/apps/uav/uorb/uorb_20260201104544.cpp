/****************************************************************************
 * apps/uav/uorb/uorb.cpp
 *
 * uORB - Implementation
 *
 * THIẾT KẾ NỘI BỘ:
 * - TopicNode: Chứa ring buffer + metadata cho một topic instance
 * - TopicRegistry: Quản lý tất cả topics trong hệ thống
 * - SubscriberHandle: Theo dõi sequence cho mỗi subscriber
 *
 * THREAD SAFETY:
 * - Mỗi TopicNode có mutex riêng
 * - Registry có mutex riêng
 * - Không deadlock vì không nested lock
 *
 * MEMORY:
 * - Tất cả static allocation
 * - Ring buffer trong TopicNode (8 entries default)
 * - Max 32 topics x 4 instances = 128 nodes
 *
 ****************************************************************************/

#include "uorb.hpp"
#include <cstring>
#include <cerrno>

namespace uorb
{

/****************************************************************************
 * TopicNode - Một instance của topic
 *
 * Chứa:
 * - Ring buffer cho messages
 * - Mutex bảo vệ
 * - Sequence counter
 ****************************************************************************/

class TopicNode
{
public:
    static constexpr size_t MAX_MSG_SIZE = 256;
    static constexpr int QUEUE_SIZE = ORB_DEFAULT_QUEUE_SIZE;

    TopicNode() : _meta(nullptr), _seq(0), _write_idx(0), _initialized(false) {}

    bool init(orb_id_t meta)
    {
        if (_initialized) return true;
        if (!meta || meta->size > MAX_MSG_SIZE) return false;

        _meta = meta;
        _seq = 0;
        _write_idx = 0;
        memset(_buffer, 0, sizeof(_buffer));

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL);
        pthread_mutex_init(&_lock, &attr);
        pthread_mutexattr_destroy(&attr);

        _initialized = true;
        return true;
    }

    void deinit()
    {
        if (_initialized) {
            pthread_mutex_destroy(&_lock);
            _initialized = false;
        }
    }

    bool publish(const void* data)
    {
        if (!_initialized || !data) return false;

        pthread_mutex_lock(&_lock);
        memcpy(&_buffer[_write_idx * MAX_MSG_SIZE], data, _meta->size);
        _write_idx = (_write_idx + 1) % QUEUE_SIZE;
        _seq++;
        pthread_mutex_unlock(&_lock);

        return true;
    }

    bool copy(void* buffer)
    {
        if (!_initialized || !buffer || _seq == 0) return false;

        pthread_mutex_lock(&_lock);
        int latest = (_write_idx == 0) ? (QUEUE_SIZE - 1) : (_write_idx - 1);
        memcpy(buffer, &_buffer[latest * MAX_MSG_SIZE], _meta->size);
        pthread_mutex_unlock(&_lock);

        return true;
    }

    uint32_t sequence() const { return _seq; }
    bool valid() const { return _initialized && _seq > 0; }
    orb_id_t meta() const { return _meta; }

private:
    orb_id_t _meta;
    pthread_mutex_t _lock;
    uint8_t _buffer[QUEUE_SIZE * MAX_MSG_SIZE];
    volatile uint32_t _seq;
    int _write_idx;
    bool _initialized;
};

/****************************************************************************
 * SubscriberHandle - Thông tin subscriber
 *
 * Mỗi subscriber có một handle riêng để track:
 * - Topic nào đang subscribe
 * - Sequence cuối cùng đã đọc
 ****************************************************************************/

struct SubscriberHandle
{
    TopicNode* node;
    uint32_t last_seq;
    bool active;
};

/****************************************************************************
 * TopicRegistry - Quản lý tất cả topics
 *
 * Singleton pattern, static storage
 ****************************************************************************/

class TopicRegistry
{
public:
    static constexpr int MAX_TOPICS = ORB_MAX_TOPICS;
    static constexpr int MAX_INSTANCES = ORB_MAX_INSTANCES;
    static constexpr int MAX_SUBSCRIBERS = 64;

    static TopicRegistry& instance()
    {
        static TopicRegistry reg;
        return reg;
    }

    // Tìm hoặc tạo topic node
    TopicNode* getOrCreate(orb_id_t meta, int inst)
    {
        if (!meta || inst < 0 || inst >= MAX_INSTANCES) return nullptr;

        pthread_mutex_lock(&_registry_lock);

        // Tìm topic đã tồn tại
        for (int i = 0; i < _num_topics; i++) {
            if (_entries[i].meta == meta && _entries[i].instance == inst) {
                pthread_mutex_unlock(&_registry_lock);
                return &_entries[i].node;
            }
        }

        // Tạo mới
        if (_num_topics >= MAX_TOPICS * MAX_INSTANCES) {
            pthread_mutex_unlock(&_registry_lock);
            return nullptr;
        }

        TopicEntry& entry = _entries[_num_topics];
        entry.meta = meta;
        entry.instance = inst;
        if (!entry.node.init(meta)) {
            pthread_mutex_unlock(&_registry_lock);
            return nullptr;
        }
        _num_topics++;

        pthread_mutex_unlock(&_registry_lock);
        return &entry.node;
    }

    // Tìm topic (không tạo mới)
    TopicNode* find(orb_id_t meta, int inst)
    {
        if (!meta || inst < 0 || inst >= MAX_INSTANCES) return nullptr;

        pthread_mutex_lock(&_registry_lock);
        for (int i = 0; i < _num_topics; i++) {
            if (_entries[i].meta == meta && _entries[i].instance == inst) {
                pthread_mutex_unlock(&_registry_lock);
                return &_entries[i].node;
            }
        }
        pthread_mutex_unlock(&_registry_lock);
        return nullptr;
    }

    // Đếm instances
    int countInstances(orb_id_t meta)
    {
        int count = 0;
        pthread_mutex_lock(&_registry_lock);
        for (int i = 0; i < _num_topics; i++) {
            if (_entries[i].meta == meta) count++;
        }
        pthread_mutex_unlock(&_registry_lock);
        return count;
    }

    // Allocate subscriber handle
    int allocSubscriber(TopicNode* node)
    {
        if (!node) return -1;

        pthread_mutex_lock(&_registry_lock);
        for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
            if (!_subscribers[i].active) {
                _subscribers[i].node = node;
                _subscribers[i].last_seq = 0;
                _subscribers[i].active = true;
                pthread_mutex_unlock(&_registry_lock);
                return i;
            }
        }
        pthread_mutex_unlock(&_registry_lock);
        return -1;
    }

    // Free subscriber handle
    void freeSubscriber(int fd)
    {
        if (fd < 0 || fd >= MAX_SUBSCRIBERS) return;
        pthread_mutex_lock(&_registry_lock);
        _subscribers[fd].active = false;
        _subscribers[fd].node = nullptr;
        pthread_mutex_unlock(&_registry_lock);
    }

    // Get subscriber handle
    SubscriberHandle* getSubscriber(int fd)
    {
        if (fd < 0 || fd >= MAX_SUBSCRIBERS) return nullptr;
        if (!_subscribers[fd].active) return nullptr;
        return &_subscribers[fd];
    }

private:
    TopicRegistry()
    {
        _num_topics = 0;
        memset(_entries, 0, sizeof(_entries));
        memset(_subscribers, 0, sizeof(_subscribers));
        pthread_mutex_init(&_registry_lock, nullptr);
    }

    struct TopicEntry {
        orb_id_t meta;
        int instance;
        TopicNode node;
    };

    pthread_mutex_t _registry_lock;
    TopicEntry _entries[MAX_TOPICS * MAX_INSTANCES];
    SubscriberHandle _subscribers[MAX_SUBSCRIBERS];
    int _num_topics;
};

/****************************************************************************
 * Public API Implementation
 ****************************************************************************/

orb_advert_t orb_advertise(orb_id_t meta, const void* data)
{
    return orb_advertise_multi(meta, data, 0);
}

orb_advert_t orb_advertise_multi(orb_id_t meta, const void* data, int instance)
{
    TopicNode* node = TopicRegistry::instance().getOrCreate(meta, instance);
    if (!node) {
        errno = ENOMEM;
        return nullptr;
    }

    if (data) {
        node->publish(data);
    }

    return static_cast<orb_advert_t>(node);
}

int orb_unadvertise(orb_advert_t handle)
{
    // Trong thiết kế này, topic tồn tại mãi mãi
    // unadvertise chỉ là no-op để tương thích API
    (void)handle;
    return 0;
}

int orb_publish(orb_id_t meta, orb_advert_t handle, const void* data)
{
    (void)meta;  // Không dùng vì handle đã có metadata

    TopicNode* node = static_cast<TopicNode*>(handle);
    if (!node || !data) {
        errno = EINVAL;
        return -1;
    }

    if (!node->publish(data)) {
        errno = EIO;
        return -1;
    }

    return 0;
}

int orb_subscribe(orb_id_t meta)
{
    return orb_subscribe_multi(meta, 0);
}

int orb_subscribe_multi(orb_id_t meta, int instance)
{
    // Tìm hoặc tạo topic (để subscribe trước khi advertise cũng được)
    TopicNode* node = TopicRegistry::instance().getOrCreate(meta, instance);
    if (!node) {
        errno = ENOMEM;
        return -1;
    }

    int fd = TopicRegistry::instance().allocSubscriber(node);
    if (fd < 0) {
        errno = EMFILE;
        return -1;
    }

    return fd;
}

int orb_unsubscribe(int fd)
{
    TopicRegistry::instance().freeSubscriber(fd);
    return 0;
}

int orb_copy(orb_id_t meta, int fd, void* buffer)
{
    (void)meta;

    SubscriberHandle* sub = TopicRegistry::instance().getSubscriber(fd);
    if (!sub || !sub->node || !buffer) {
        errno = EINVAL;
        return -1;
    }

    if (!sub->node->copy(buffer)) {
        errno = EAGAIN;
        return -1;
    }

    sub->last_seq = sub->node->sequence();
    return 0;
}

int orb_check(int fd, bool* updated)
{
    if (!updated) {
        errno = EINVAL;
        return -1;
    }

    SubscriberHandle* sub = TopicRegistry::instance().getSubscriber(fd);
    if (!sub || !sub->node) {
        errno = EBADF;
        return -1;
    }

    *updated = (sub->node->sequence() != sub->last_seq);
    return 0;
}

bool orb_exists(orb_id_t meta, int instance)
{
    TopicNode* node = TopicRegistry::instance().find(meta, instance);
    return (node != nullptr && node->valid());
}

int orb_group_count(orb_id_t meta)
{
    return TopicRegistry::instance().countInstances(meta);
}

} // namespace uorb
