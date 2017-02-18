#include "buffer.h"

void Buffer_clear(void)
{
	// clear the curve
	_num_items = 0;
    _head = 0;
}

void Buffer_push_back( const float item )
{
    // determine position of new item
    uint8_t tail = _head + _num_items;
    if( tail >= SIZE )
    {
        tail -= SIZE;
    }

    // add item to buffer
    _buff[tail] = item;

    // increment number of items
    if( _num_items < SIZE )
    {
        _num_items++;
    }
    else
    {
        // no room for new items so drop oldest item
        _head++;
        if( _head >= SIZE )
        {
            _head = 0;
        }
    }
}

float Buffer_pop_front(void)
{
	float result;

	// return zero if buffer is empty
	if( _num_items == 0 )
	{
		return 0;
	}

	// get next value in buffer
    result = _buff[_head];

    // increment to next point
    _head++;
    if( _head >= SIZE )
        _head = 0;

    // reduce number of items
    _num_items--;

    // return item
    return result;
}

float Buffer_peek(uint8_t position)
{
    uint8_t j = _head + position;

    // return zero if position is out of range
    if( position >= _num_items )
    {
        return 0;
    }

    // wrap around if necessary
    if( j >= SIZE )
        j -= SIZE;

    // return desired value
    return _buff[j];
}

float Buffer_front(void)
{
	return Buffer_peek(0);
}


