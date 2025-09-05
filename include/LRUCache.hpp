#include <list>
#include <unordered_map>

template <typename K, typename V = K> class LRUCache {

private:
  std::list<K> items;
  std::unordered_map<K, std::pair<V, typename std::list<K>::iterator>>
      keyValuesMap;
  int csize;

public:
  LRUCache(int s) : csize(s) {
    if (csize < 1)
      csize = 10;
  }

  void set(const K key, const V value) {
    auto pos = keyValuesMap.find(key);
    if (pos == keyValuesMap.end()) {
      items.push_front(key);
      keyValuesMap[key] = {value, items.begin()};
      if (keyValuesMap.size() > csize) {
        keyValuesMap.erase(items.back());
        items.pop_back();
      }
    } else {
      items.erase(pos->second.second);
      items.push_front(key);
      keyValuesMap[key] = {value, items.begin()};
    }
  }

  bool get(const K key, V &value) {
    auto pos = keyValuesMap.find(key);
    if (pos == keyValuesMap.end())
      return false;
    items.erase(pos->second.second);
    items.push_front(key);
    keyValuesMap[key] = {pos->second.first, items.begin()};
    value = pos->second.first;
    return true;
  }
};
