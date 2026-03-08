#pragma once


template <typename T>
class Buffer {
  private:
    T* buffer;     // Pointer to the array
    int capacity;  // Maximum size
    int head = 0;
    int tail = 0;
    int count = 0;

  public:
    Buffer(int size) : capacity(size) {
      buffer = new T[capacity]; 
    }

    ~Buffer() {
      delete[] buffer;
    }

    bool push(const T& value) {
      if (count == capacity) return false;
      buffer[head] = value;
      head = (head + 1) % capacity;
      count++;
      return true;
    }

    T pop() {
      if (count == 0) return T();
      T value = buffer[tail];
      tail = (tail + 1) % capacity;
      count--;
      return value;
    }

    T get(int index) const {
        if (index >= count) {
            return T(); // Out of bounds, return default
        }
        
        // Calculate the actual position in the internal array
        int actualIndex = (tail + index) % capacity;
        return buffer[actualIndex];
    }

    void pushOverwrite(const T& value) {
      if (count == capacity) {
        tail = (tail + 1) % capacity;
      } else {
        count++;
      }
      
      buffer[head] = value;
      head = (head + 1) % capacity;
    }

    int size() const { return count; }

    void fill(const T& value) {
      for (int i = 0; i < capacity; i++)
      {
        this->pushOverwrite(0.0);
      }
    }
  };
