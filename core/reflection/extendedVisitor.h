#ifndef EXTENDEDVISITOR_H
#define EXTENDEDVISITOR_H
/**
 *
 *  Library has both static and dynamic polimorphism serializers.
 *
 *  Former is implemented in classes using accept template method
 *  Latter with the imterface below.
 *
 **/

namespace corecvs {

class Serializable {

};

/**
 *
 *  Base class for visitors that go beyond simple template based functions
 **/
class ExtendedVisitor
{
public:
    ExtendedVisitor();

};


} // namespace

#endif // EXTENDEDVISITOR_H
