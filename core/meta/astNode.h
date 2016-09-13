#ifndef ASTNODE_H
#define ASTNODE_H

/**
 * \file astNode.h
 * \brief Abstract Syntax Tree Node
 *
 * \ingroup cppcorefiles
 */
#include "global.h"

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <unordered_map>

namespace corecvs {

class ASTNodeInt;

class ASTContext
{
public:
    static ASTContext *MAIN_CONTEXT;

    std::vector<ASTNodeInt *> nodes;

    ~ASTContext()
    {
        //SYNC_PRINT(("ASTContext::~ASTContext(): there is %d garbage remaining\n", (int)nodes.size()));
        for (auto it = nodes.begin(); it != nodes.end(); ++it)
        {
            delete_safe(*it);
        }
    }

    /**
     * Mark and sweep gc
     * We reuse
     **/
    void clear();
    void mark(ASTNodeInt *node);
    void sweep();


};

struct ASTRenderDec {
    const char *ident;
    const char *lbr;

    bool genParameters;
    std::ostream &output = std::cout;

    std::unordered_map<uint64_t, ASTNodeInt *> *cse = NULL;

    ASTRenderDec(const char *ident, const char *lbr, bool genParameters) :
        ident(ident), lbr(lbr), genParameters(genParameters)
    {}

    ASTRenderDec(const char *ident, const char *lbr, bool genParameters,  std::ostream &output ) :
        ident(ident), lbr(lbr), genParameters(genParameters), output(output)
    {}


};

/**
 *  Internal data structure for ASTTree - it records all operations with ASTNode.
 *
 *  We expect this structure to be immutable. So it is safe to make DAG like structures.
 *
 **/
class ASTNodeInt
{
public:

    void _init()
    {
        if (ASTContext::MAIN_CONTEXT == NULL)
            return;

        /* We need to check if this memeory management s*/
        ASTContext::MAIN_CONTEXT->nodes.push_back(this);
    }

    ASTNodeInt() {
        _init();
    }

    enum Operator {
        OPREATOR_ID,
        OPREATOR_NUM,

        OPERATOR_BINARY = OPREATOR_NUM,
        OPERATOR_ADD,
        OPERATOR_SUB,
        OPERATOR_MUL,
        OPERATOR_DIV,
        OPERATOR_BINARY_LAST = OPERATOR_DIV,

        /**/
        OPERATOR_POW,
        /**/
        OPERATOR_SIN,
        OPERATOR_COS,

        OPERATOR_USER,

        OPERATOR_LAST
    };

    static inline const char *getName(const Operator &value)
    {
        switch (value)
        {
             case OPREATOR_ID  : return "ID"; break ;
             case OPREATOR_NUM : return "value"; break ;

             case OPERATOR_ADD : return "add"; break ;
             case OPERATOR_SUB : return "sub"; break ;
             case OPERATOR_MUL : return "mul"; break ;
             case OPERATOR_DIV : return "div"; break ;

            /**/
             case OPERATOR_POW : return "pow"; break ;
            /**/
             case OPERATOR_SIN : return "sin"; break ;
             case OPERATOR_COS : return "cos"; break ;

             case OPERATOR_USER : return "user"; break ;

             case OPERATOR_LAST : return "R"; break ;
        }
        return "Not in range";
    }

    static inline int getPriority(const Operator &value)
    {
        switch (value)
        {
             case OPREATOR_ID  : return 40; break ;
             case OPREATOR_NUM : return 40; break ;

             case OPERATOR_ADD : return 10; break ;
             case OPERATOR_SUB : return 10; break ;
             case OPERATOR_MUL : return 20; break ;
             case OPERATOR_DIV : return 20; break ;

            /**/
             case OPERATOR_POW : return 30; break ;
            /**/
             case OPERATOR_SIN : return 30; break ;
             case OPERATOR_COS : return 30; break ;

             case OPERATOR_USER: return  5; break ;

             case OPERATOR_LAST : return 0; break ;
        }
        return 0;
    }



    ASTNodeInt(Operator _op, ASTNodeInt *_left = NULL, ASTNodeInt *_right = NULL) :
        op   (_op),
        left (_left),
        right(_right)
    {
        _init();
    }

    explicit ASTNodeInt(double _value) :
        op   (OPREATOR_NUM),
        val  (_value),
        left (NULL),
        right(NULL)
    {
        _init();
    }

    explicit ASTNodeInt(const char *_name) :
        op   (OPREATOR_ID),
        val  (0),
        name (_name),
        left (NULL),
        right(NULL)
    {
        _init();
    }

    ~ASTNodeInt()
    {
        if (ASTContext::MAIN_CONTEXT == NULL)
            return;
        //delete_safe(payload); /* virual destuctor problem here. so far nobody cares */
#if 0
        auto &vec = owner->nodes;
        vec.erase(std::remove(vec.begin(), vec.end(), this), vec.end());
#endif
    }

    Operator op;

    /*Context *owner;*/

    double val;
    std::string name;

    ASTNodeInt *left  = NULL;
    ASTNodeInt *right = NULL;

    /* We can unite the fields below */
    uint64_t hash = 0;
    uint32_t height = 0;
    //void *payload = NULL;

    /* Common subexpresstion related variables. Could be moved to payload */
    int cseCount = 0;
    int cseName = 0;

    /* Flag used for mark and sweep garbage collection */
    uint16_t markFlag = 0;

    bool isBinary();

    bool isUnary();

    static ASTRenderDec identSymDefault;
    static ASTRenderDec identSymLine;

    void codeGenCpp (const std::string &name, ASTRenderDec &identSym = identSymDefault);
    void codeGenCpp (int ident , ASTRenderDec &identSym = identSymDefault);

    /**
     * NB This function actually modifies the tree elements.
     **/
    void extractConstPool(const std::string &poolname, std::unordered_map<double, std::string> &pool);
    /* Computes hash and height */
    void rehash();
    void cseR(std::unordered_map<uint64_t, ASTNodeInt *> &cse);


    size_t memoryFootprint();
    void print();
    void getVars(std::vector<std::string> &result);

    ASTNodeInt* derivative(const std::string &var);
    ASTNodeInt* compute(const std::map<std::string, double>& bind = std::map<std::string, double>());


    /*unsafe stuff you can destroy subtree of an another tree by accident*/
    static void deleteSubtree(ASTNodeInt *tree);
    void deleteChildren();

};


class ASTNode {
public:
    ASTNodeInt *p;

    ASTNode() :
        p(new ASTNodeInt("UDEF"))
    {}

    explicit ASTNode(int _value) :
        p(new ASTNodeInt((double)_value))
    {
//        SYNC_PRINT(("ASTNode(%i): with id %p\n", _value, static_cast<void*>(p)));
    }

    explicit ASTNode(double _value) :
        p(new ASTNodeInt(_value))
    {
//        SYNC_PRINT(("ASTNode(%lf): with id %p\n", _value, static_cast<void*>(p)));
    }

    explicit ASTNode(const char *_value) :
        p(new ASTNodeInt(_value))
    {
//        SYNC_PRINT(("ASTNode(\"%s\"): with id %p\n", _value, static_cast<void*>(p)));
    }

    ASTNode(ASTNodeInt::Operator _op, const ASTNode &_left, const ASTNode &_right) :
        p(new ASTNodeInt(_op, _left.p, _right.p))
    {
    }


//    friend ASTNode operator *(ASTNode left, ASTNode right);


};


inline ASTNode operator *(const ASTNode &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_MUL, left, right);
}

inline ASTNode operator +(const ASTNode &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_ADD, left, right);
}

inline ASTNode operator +=(ASTNode &left, const  ASTNode &right)
{
    left.p = new ASTNodeInt(ASTNodeInt::OPERATOR_ADD, left.p, right.p);
    return left;
}

inline ASTNode operator -(const ASTNode &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_SUB, left, right);
}

inline ASTNode operator /(const ASTNode &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_DIV, left, right);
}

inline ASTNode sqrt(const ASTNode &left)
{
    return ASTNode(ASTNodeInt::OPERATOR_POW, left, ASTNode(0.5));
}

/* King Midas style operators */

inline ASTNode operator *(const ASTNode &left, const double &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_MUL, left, ASTNode(right));
}

inline ASTNode operator *(const double &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_MUL, ASTNode(left), right);
}

inline ASTNode operator +(const ASTNode &left, const double &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_ADD, left, ASTNode(right));
}

inline ASTNode operator +(const double &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_ADD, ASTNode(left), right);
}


inline ASTNode operator -(const ASTNode &left, const double &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_SUB, left, ASTNode(right));
}

inline ASTNode operator -(const double &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_SUB, ASTNode(left), right);
}

inline ASTNode operator -(const ASTNode &right)
{
    return (0 - right);
}

inline ASTNode operator /(const double &left, const ASTNode &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_DIV,  ASTNode(left), right);
}


inline ASTNode operator /(const ASTNode &left, const double &right)
{
    return ASTNode(ASTNodeInt::OPERATOR_DIV, left,  ASTNode(right));
}


} //namespace corecvs

#endif // ASTNODE_H
