#ifndef _COUNTER_H_
#define _COUNTER_H_

#include <stdexcept>
#include <vector>

class Counter
{
  public:
    Counter() { }

    Counter(const std::vector<unsigned>& max)
    {
      reset(max);
    }

    unsigned size() const { return counter.size(); }

    /// Resets the counter with the new maximum
    void reset(const std::vector<unsigned>& max)
    {
      this->max = max;
      counter = std::vector<unsigned>(max.size(), 0);
    }

    /// Resets the counter
    void reset()
    {
      counter = std::vector<unsigned>(max.size(), 0);
    }

    /// Checks to see whether the counter is full
    bool full() const 
    { 
      for (unsigned i=0; i< max.size(); i++)
        if (max[i] != counter[i])
          return false;
      return true;
    } 
    
    Counter& operator++(int)
    {
      int idx = counter.size()-1;
      while (true)
      {
        counter[idx]++;
        if (counter[idx] > max[idx])
        {
          counter[idx] = 0;
          idx--;
          if (idx == -1)
            throw std::runtime_error("Attempted to increment a full counter!");
        }
        else
          break;
      }
    }

  const std::vector<unsigned>& get() const { return counter; }

  private:
    std::vector<unsigned> counter;
    std::vector<unsigned> max;
};

#endif

