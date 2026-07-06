#ifndef __MESSAGE_QUE_H__
#define __MESSAGE_QUE_H__

#include <stdio.h>
#include <list>
#include <mutex>

template<typename T>
class TMessageQue
{
public:
	
	// 向队列尾部添加元素
	void push_back(T const& msg)
	{
		m_mutex.lock();
		m_list.push_back(msg);
		m_mutex.unlock();
	}
	
	// 返回队列头部元素，同时删除队列头部元素
	bool get(T& msg)
	{
		bool nRet = false;
		m_mutex.lock();
		if (!m_list.empty())
		{
			msg = m_list.front();
			m_list.pop_front();
			nRet = true;
		}
		m_mutex.unlock();

		return nRet;
	}
	
	// 返回队列大小
	int size()
	{
		m_mutex.lock();
		int size =  (int)m_list.size();
		m_mutex.unlock();

		return size;
	}

	// 清空队列
	void clear()
	{
		m_mutex.lock();
		m_list.clear();
		m_mutex.unlock();
	}
private:
	
	std::mutex			    m_mutex;
	std::list<T>			m_list;
};

#endif //__MESSAGE_QUE_H__
