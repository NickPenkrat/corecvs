#ifndef G8BUFFER_H_
#define G8BUFFER_H_
/**
 * \file g8Buffer.h
 * \brief Add Comment Here
 *
 * \ingroup cppcorefiles
 * \date Jul 6, 2010
 * \author alexander
 */



#include "abstractBuffer.h"
#include "abstractContiniousBuffer.h"
#include "stdio.h"
#include "g12Buffer.h"

namespace core3vi {

typedef AbstractContiniousBuffer<uint8_t, int32_t> G8BufferBase;

class G8Buffer : public G8BufferBase
{
public:

    G8Buffer(int32_t h, int32_t w, bool shouldInit = true) : G8BufferBase (h, w, shouldInit) {};
    G8Buffer(Vector2d<int32_t> size, bool shouldInit = true) : G8BufferBase (size, shouldInit) {};


    G8Buffer(G8Buffer &that) : G8BufferBase (that) {};
    G8Buffer(G8Buffer *that) : G8BufferBase (that) {};

    G8Buffer(G8Buffer *src, int32_t x1, int32_t y1, int32_t x2, int32_t y2) :
        G8BufferBase(src, x1, y1, x2, y2) {};

    static G8Buffer* FromG12Buffer(core3vi::G12Buffer *input);
    //G8Buffer(int32_t h, int32_t w, uint16_t *data) : G8BufferBase(h, w, data) {};
};


} //namespace core3vi
#endif /* G8BUFFER_H_ */

