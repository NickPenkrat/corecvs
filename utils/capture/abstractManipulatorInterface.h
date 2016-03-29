#ifndef ABSTARCTMANIPULATORINTERFACE_H
#define ABSTARCTMANIPULATORINTERFACE_H


class AbstractManipulatorInterface
{
public:
    virtual void configureManipulator() = 0;
    virtual bool disableManipulator() = 0;
    virtual bool enableManipulator() = 0;
    virtual bool manipulatorGetReady() = 0;
    virtual bool moveToPosition(int delta) = 0;
    virtual bool moveToInitialPosition() = 0;
};

#endif // ABSTARCTMANIPULATORINTERFACE_H
