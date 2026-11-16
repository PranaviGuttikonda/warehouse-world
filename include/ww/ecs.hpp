#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace ww {

using Entity = std::uint32_t;
inline constexpr Entity kNullEntity = std::numeric_limits<Entity>::max();

/// Type-erased base so the Registry can remove an entity from every pool.
class IPool {
public:
    virtual ~IPool() = default;
    virtual void remove(Entity e) = 0;
    virtual bool has(Entity e) const = 0;
};

/// Sparse-set storage for one component type.
///
/// Given: entities with small integer ids.
/// Approach: sparse_[entity] -> index into dense arrays. Components sit
/// contiguously in data_, so systems iterate them cache-friendly. Removal is
/// swap-with-last + pop, O(1), and keeps the arrays dense.
template <typename T>
class ComponentPool final : public IPool {
public:
    T& add(Entity e, T value) {
        assert(!has(e) && "component already present");
        if (e >= sparse_.size()) sparse_.resize(e + 1, kEmpty);
        sparse_[e] = static_cast<std::uint32_t>(dense_.size());
        dense_.push_back(e);
        data_.push_back(std::move(value));
        return data_.back();
    }

    void remove(Entity e) override {
        if (!has(e)) return;
        std::uint32_t idx = sparse_[e];
        std::uint32_t last = static_cast<std::uint32_t>(dense_.size() - 1);
        if (idx != last) {
            dense_[idx] = dense_[last];
            data_[idx] = std::move(data_[last]);
            sparse_[dense_[idx]] = idx;
        }
        dense_.pop_back();
        data_.pop_back();
        sparse_[e] = kEmpty;
    }

    bool has(Entity e) const override { return e < sparse_.size() && sparse_[e] != kEmpty; }

    T& get(Entity e) {
        assert(has(e));
        return data_[sparse_[e]];
    }
    const T& get(Entity e) const {
        assert(has(e));
        return data_[sparse_[e]];
    }

    std::size_t size() const { return dense_.size(); }
    const std::vector<Entity>& entities() const { return dense_; }
    std::vector<T>& data() { return data_; }

private:
    static constexpr std::uint32_t kEmpty = std::numeric_limits<std::uint32_t>::max();
    std::vector<std::uint32_t> sparse_;
    std::vector<Entity> dense_;
    std::vector<T> data_;
};

/// Owns entities and one pool per component type.
class Registry {
public:
    /// Given: nothing. Approach: reuse a freed id if any, else the next id.
    Entity create() {
        if (!free_.empty()) {
            Entity e = free_.back();
            free_.pop_back();
            alive_[e] = true;
            return e;
        }
        alive_.push_back(true);
        return static_cast<Entity>(alive_.size() - 1);
    }

    void destroy(Entity e) {
        if (!alive(e)) return;
        for (auto& [_, pool] : pools_) pool->remove(e);
        alive_[e] = false;
        free_.push_back(e);
    }

    bool alive(Entity e) const { return e < alive_.size() && alive_[e]; }

    template <typename T>
    T& add(Entity e, T value) {
        assert(alive(e));
        return pool<T>().add(e, std::move(value));
    }
    template <typename T>
    T& get(Entity e) { return pool<T>().get(e); }
    template <typename T>
    bool has(Entity e) { return pool<T>().has(e); }
    template <typename T>
    void remove(Entity e) { pool<T>().remove(e); }

    template <typename T>
    ComponentPool<T>& pool() {
        auto& slot = pools_[std::type_index(typeid(T))];
        if (!slot) slot = std::make_unique<ComponentPool<T>>();
        return static_cast<ComponentPool<T>&>(*slot);
    }

    /// Read-only access. A type never added yields an empty pool.
    template <typename T>
    const ComponentPool<T>& pool() const {
        static const ComponentPool<T> empty;
        auto it = pools_.find(std::type_index(typeid(T)));
        return it == pools_.end() ? empty : static_cast<const ComponentPool<T>&>(*it->second);
    }
    template <typename T>
    const T& get(Entity e) const { return pool<T>().get(e); }

private:
    std::vector<bool> alive_;
    std::vector<Entity> free_;
    std::unordered_map<std::type_index, std::unique_ptr<IPool>> pools_;
};

}  // namespace ww
