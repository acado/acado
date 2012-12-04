// Object pool, a template

#ifndef ACADO_TOOLKIT_OBJECT_POOL
#define ACADO_TOOLKIT_OBJECT_POOL

#include <map>
#include <vector>
#include <utility>

/** An object pool class that creates and reuses objects of an arbitrary type. */
template
<
	/** Type of objects to store in the pool */
	typename T,
	/** Comparator for object stored in the pool */
	typename C = std::less< T >
>
class ObjectPool
{
public:
	/** Default constructor */
	ObjectPool()
	{}
	
	/** Default destructor */
	~ObjectPool()
	{}
	
	/** This function enables one to explicitly put an object into the pool.
	 *
	 *  In case the object is already in the pool, the function returns false.
	 *  Otherwise, it returns true.
	 *  */
	bool add(const T& obj)
	{
		typename poolMap::const_iterator it = pool.find( obj );
		if (it == pool.end())
		{
			pool.insert( std::make_pair(obj, true) );

			return true;
		}

		return false;
	}

	bool busy( void )
	{
		if (pool.size() == 0)
			return true;

		typename poolMap::const_iterator it = pool.begin();
		for (; it != pool.end(); ++it)
			if (it->second == false)
				return false;

		return true;
	}

	/** When user code demands an object, pool first looks if there is any spare
	 *  object to return. In case there is not any available object in the pool,
	 *  a new object will be created and returned.
	 * */
	bool acquire(T& obj)
	{
		if ( pool.size() )
		{
			typename poolMap::iterator it = pool.begin();
			for (; it != pool.end(); ++it)
			{
				if (it->second == false)
					break;
			}
			
			if (it != pool.end())
			{
				obj = it->first;
				it->second = true;

				return true;
			}
		}	
			
		return false;
	}
	
	/** This function releases an object in the pool */
	bool release(const T& obj)
	{
		typename poolMap::iterator it = pool.find( obj );
		if (it != pool.end())
		{
			it->second = false;

			return true;
		}

		return false;
	}
	
	/** This function return a vector containing all objects in the pool */
	std::vector< T > getPool() const
	{
		std::vector< T > v;

		typename poolMap::const_iterator it = pool.begin();
		for (; it != pool.end(); ++it)
			v.push_back( it->first );

		return v;
	}

	/** Number of objects in the pool */
	unsigned size( void )
	{
		return pool.size();
	}

private:
	typedef std::map<T, bool, C> poolMap;

	poolMap pool;
};

#endif // ACADO_TOOLKIT_OBJECT_POOL
