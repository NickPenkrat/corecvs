#ifndef COMMENTFILTER_H
#define COMMENTFILTER_H

#include <iostream>
#include <sstream>
#include <streambuf>

#if 0
class CommentFilter : public std::streambuf
{
public:
    std::streambuf* mSource;

    commentFilter(std::streambuf* source) :
        mSource(source)
    {

    }
};

class CommentFilterStream : public std::istream
{
    CommentFilter buf;

};

#endif

#endif // COMMENTFILTER_H
