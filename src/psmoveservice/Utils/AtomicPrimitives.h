#ifndef ATOMIC_PRIMITIVES_H
#define ATOMIC_PRIMITIVES_H

#include <atomic>
#include <assert.h>

// Triple buffered lock free atomic generic object
// Inspired by: https://gist.github.com/andrewrk/03c369c82de4701625e3
template<typename t_object_type>
class AtomicObject 
{
public:
    AtomicObject() 
	{
        m_inUseIndex = 0;
        m_activeIndex = 0;
        m_write_index = -1;

		for (int i = 0; i < 3; ++i)
		{
			m_objects[i] = new t_object_type;
		}
    }
    
	virtual ~AtomicObject() 
	{
		for (int i = 0; i < 3; ++i)
		{
			delete m_objects[i];
		}
	}

	void storeValue(const t_object_type &object)
	{
		t_object_type *objectSlot= writeBegin();
		*objectSlot= object;
		writeEnd();
	}

	void fetchValue(t_object_type &out_object)
	{
		out_object= *getReadPtr();
	}

protected:
    t_object_type *writeBegin() 
	{
        assert(m_write_index == -1);
        int in_use_index = m_inUseIndex.load();
        int active_index = m_activeIndex.load();

		if (in_use_index != 0 && active_index != 0)
		{
            m_write_index = 0;
		}
		else if (in_use_index != 1 && active_index != 1)
		{
            m_write_index = 1;
		}
		else
		{
            m_write_index = 2;
		}

        return m_objects[m_write_index];
    }

    void writeEnd() 
	{
        assert(m_write_index != -1);
        m_activeIndex.store(m_write_index);
        m_write_index = -1;
    }

    t_object_type *getReadPtr() 
	{
        m_inUseIndex.store(m_activeIndex.load());

        return m_objects[m_inUseIndex];
    }

private:
    t_object_type* m_objects[3];
    std::atomic_int m_inUseIndex;
    std::atomic_int m_activeIndex;
    int m_write_index;

    AtomicObject(const AtomicObject &copy) = delete;
    AtomicObject &operator=(const AtomicObject &copy) = delete;
};

#endif // ATOMIC_PRIMITIVES_H
