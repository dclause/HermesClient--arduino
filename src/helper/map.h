#ifndef HERMESCLIENT_MAP_H
#define HERMESCLIENT_MAP_H

#include <Arduino.h>
#include "debugger.h"

/**
 * Defines an associative key/value element to be used in KeyValueMap.
 *
 * @tparam Key      the key element type (ex. int, string, etc...)
 * @tparam Value    the value element type (ex. AbstractCommand, etc...)
 */
template<typename Key, typename Value>
struct KeyValuePair {
    Key key;
    Value value;
    KeyValuePair<Key, Value> *next;
};

/**
 * Defines a FIFO-type chained list of associative key/value pairs.
 *
 * @tparam Key      the key element type (ex. int, string, etc...)
 * @tparam Value    the value element type (ex. AbstractCommand, etc...)
 */
template<typename Key, typename Value>
class KeyValueMap {
private:
    KeyValuePair<Key, Value> *head_;
    uint8_t size_ = 0;

public:
    KeyValueMap() : head_(NULL), size_(0) {}

    ~KeyValueMap() { this->clear(); }

    /**
     * Returns the number of elements in the map.
     */
    uint8_t count() {
        return this->size_;
    }

    /**
     * Returns the map head.
     */
    KeyValuePair<Key, Value> *getHead() {
        return this->head_;
    }

    /**
     * Deletes all elements in the list.
     */
    void clear() {
        KeyValuePair<Key, Value> *nodeToDelete;
        while (this->head_) {
            nodeToDelete = this->head_;
            this->head_ = this->head_->next;
            delete nodeToDelete;
        }
        this->size_ = 0;
    }

    /**
     * Adds an element at the head of the list.
     *
     * @warning
     * If an element with the same key exists, the addition is rejected and the return is `false`.
     *
     * @param key (Key)
     * @param value
     * @return bool If the element has been added to the list.
     */
    bool add(const Key key, const Value value) {
        if (this->getPosition(key) != -1) {
            return false;
        }
        auto *newNode = new KeyValuePair<Key, Value>;
        newNode->key = key;
        newNode->value = value;
        newNode->next = this->head_;
        this->head_ = newNode;
        this->size_ += 1;
        return true;
    }

    /**
     * Removes an element from the list.
     *
     * @param key
     * @return bool If the element has been removed from the list.
     */
    bool remove(Key key) {
        KeyValuePair<Key, Value> *current = this->head_;
        while (current) {
            if (current->next->key == key) {
                break;
            }
            current = current->next;
        }
        if (current) {
            KeyValuePair<Key, Value> *toBeDeleted = current->next;
            current->next = toBeDeleted->next;
            this->size_ -= 1;
            delete toBeDeleted;
            return true;
        }
        return false;
    }

    /**
     * Returns the element which index is `key`.
     * Overrides get operator using the [] syntax.
     *
     * @example
     *      Value value = my_list[my_key];`
     *
     * @param key
     * @return The element or NULL.
     */
    KeyValuePair<Key, Value> *operator[](Key key) {
        KeyValuePair<Key, Value> *current = this->head_;
        while (current) {
            if (current->key == key) {
                return current;
            }
            current = current->next;
        }
        return NULL;
    }

    /**
     * Returns the element which index is `key`.
     * Overrides get operator using the [] syntax.
     *
     * @note
     * This method is identical to using the map[key] operator.
     *
     * @example
     *      Value value = my_list[my_key];`
     *
     * @param key
     * @return The element or NULL.
     */
    Value getValue(Key const key) {
        KeyValuePair<Key, Value> *node = (*this)[key];
        if (node != NULL)
            return node->value;
        return NULL;
    }

    /**
     * Returns the element at i-th position.
     *
     * @param key
     * @return The pair or NULL.
     */
    KeyValuePair<Key, Value> *get(uint8_t position) {
        if (position < 0 || position >= this->size_) {
            return NULL;
        }
        KeyValuePair<Key, Value> *current = this->head_;
        for (uint8_t i = 0; i < position; i++) {
            current = current->next;
        }
        return current;
    }

    /**
     * Gets the position of the element with the given key.
     *
     * @param key
     * @return the position, or `-1` if not found.
     */
    int getPosition(Key const key) {
        KeyValuePair<Key, Value> *current = this->head_;
        for (uint8_t i = 0; i < this->size_; i++) {
            if (current->key == key) {
                return i;
            }
            current = current->next;
        }
        return -1;
    }

    /**
     * Converts to string for debug purpose.
     *
     * @return String
     */
    operator String() {
        String output = "Debug map (" + String(this->count()) + " elements):\n";
        KeyValuePair<Key, Value> *current = this->head_;
        for (uint8_t i = 0; i < this->size_; i++) {
            output += "  > " + String(current->value) + '\n';
            current = current->next;
        }
        return output;
    }
};

#endif // HERMESCLIENT_MAP_H
