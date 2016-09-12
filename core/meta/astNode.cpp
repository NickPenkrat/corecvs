#include <iostream>
#include <cmath>
#include "astNode.h"

namespace corecvs {

using std::endl;
using std::cout;

ASTContext *ASTContext::MAIN_CONTEXT = NULL;

ASTRenderDec ASTNodeInt::identSymDefault = {" ", "\n", true};
ASTRenderDec ASTNodeInt::identSymLine    = {"", "", true};


bool ASTNodeInt::isBinary() {
    return (op > OPERATOR_BINARY && op <= OPERATOR_BINARY_LAST);
}

bool ASTNodeInt::isUnary() {
    return (op == OPERATOR_POW || op == OPERATOR_SIN || op == OPERATOR_COS);
}

void ASTNodeInt::codeGenCpp(const std::string &name, ASTRenderDec &identSym)
{
    if (!identSym.genParameters) {
        printf("double %s() {\n", name.c_str());
    } else {
        std::vector<std::string> params;
        getVars(params);

        printf("double %s(", name.c_str());

        for (size_t i = 0; i < params.size(); i++)
        {
           printf("%sdouble %s", (i == 0) ? "" : ", ", params[i].c_str());
        }
        printf(") {\n");

    }
    printf("return (%s", identSym.lbr);
    codeGenCpp(1, identSym);
    printf("%s);\n", identSym.lbr);
    printf("}\n");
}

void ASTNodeInt::codeGenCpp(int ident, ASTRenderDec &identSym)
{
    std::ostream &output = identSym.output;

    for (int i = 0; i < ident; ++i) {
        output << identSym.ident;
    }

    if (op == OPREATOR_ID) {
        output << name.c_str();
        return;
    }

    if (op == OPREATOR_NUM) {
        output.precision(std::numeric_limits<double>::digits10 + 3);
        if (val >= 0) {
            output << val;
        } else {
            output << "(" << val << ")";
        }
        return;
    }
    /* Here we can check for CSE */
    if (identSym.cse != NULL && hash != 0)
    {
        auto it = identSym.cse->find(hash);
        if (it != identSym.cse->end() && (*it).second->cseCount > 0)
        {
            int cseName = (*it).second->cseName;

            output << "cse" << std::hex << cseName << std::dec;
            return;
        }
    }


    if (op > OPERATOR_BINARY && op <= OPERATOR_BINARY_LAST)
    {
        int currentPrior = getPriority(op);
        int leftPrior = getPriority(left->op);

        if (currentPrior >= leftPrior)
            output << "(" << identSym.lbr;
        left->codeGenCpp(ident + 1, identSym);
        if (currentPrior >= leftPrior)
            output << ")" << identSym.lbr;

        output << identSym.lbr;

        for (int i = 0; i < ident; ++i) {
            output << identSym.ident;
        }
        switch (op) {
            case OPERATOR_ADD : output << "+" ; break;
            case OPERATOR_SUB : output << "-" ; break;
            case OPERATOR_MUL : output << "*" ; break;
            case OPERATOR_DIV : output << "/" ; break;
            default           : output << " UNSUPPORTED " ; break;
        }

        int rightPrior = getPriority(right->op);
        if (currentPrior >= rightPrior)
            output << "(" << identSym.lbr;
        //output <<  identSym.lbr;
        right->codeGenCpp(ident + 1, identSym);
        if (currentPrior >= rightPrior)
            output << ")" << identSym.lbr;

        output << identSym.lbr;
        for (int i = 0; i < ident; ++i) {
            output << identSym.ident;
        }
        return;
    }
    if (op == OPERATOR_POW)
    {
        output <<  identSym.lbr;
        printf("pow(");
        output <<  identSym.lbr;
        left->codeGenCpp(ident + 1, identSym);
        output << ",";
        right->codeGenCpp(ident + 1, identSym);
        output <<  identSym.lbr;
        output << ")";
        output << identSym.lbr;

    }

}

void ASTNodeInt::extractConstPool(const std::string &poolname, std::unordered_map<double, std::string> &pool)
{
    if (op == OPREATOR_NUM)
    {
        //cout << "Checking " << val << " to constpool" << endl;
        auto it = pool.find(val);
        if (it == pool.end()) {
            int id = pool.size();
            char str[1000];
            snprintf2buf(str, "%s%d", poolname.c_str(), id);
            op = OPREATOR_ID;
            name = str;
            pool[val] = str;
            //cout << "Adding " << val << " to constpool" << endl;
        } else {
            op = OPREATOR_ID;
            name = (*it).second;
        }
    }

    if (isBinary())
    {
        left ->extractConstPool(poolname, pool);
        right->extractConstPool(poolname, pool);
    }

    if (isUnary())
    {
        left->extractConstPool(poolname, pool);
    }
}

void ASTNodeInt::cseR(std::unordered_map<uint64_t, ASTNodeInt *> &cse)
{
    auto it = cse.find(hash);
    if (it == cse.end()) {
        /* We are not intested of making cse out of consts and ids */
        if (isBinary())
        {
            left ->cseR(cse);
            right->cseR(cse);
            cse[hash] = this;
        }

        if (isUnary())
        {
            left->cseR(cse);
            cse[hash] = this;
        }
    } else {
        (*it).second->cseCount++;
        if (isBinary())
        {
            left ->cseR(cse);
            right->cseR(cse);
        }

        if (isUnary())
        {
            left->cseR(cse);
        }
    }
}

size_t ASTNodeInt::memoryFootprint()
{
    if (op == OPREATOR_ID)
    {
        return sizeof(ASTNodeInt);
    }

    if (op == OPREATOR_NUM)
    {
        return sizeof(ASTNodeInt);
    }

    if (isBinary())
    {

        return sizeof(ASTNodeInt) + left->memoryFootprint() + right->memoryFootprint();
    }

    if (isUnary())
    {
        return sizeof(ASTNodeInt) + left->memoryFootprint();
    }

}

void ASTNodeInt::rehash()
{
    cseCount = 0;
    cseName = 0;

    if (op == OPREATOR_ID)
    {
        hash = std::hash<std::string>{}(name);
        height = 1;
        return;
    }

    if (op == OPREATOR_NUM)
    {
        hash = std::hash<double>{}(val);
        height = 1;
        return;
    }

    if (isBinary())
    {
        left->rehash();
        right->rehash();

        hash = left->hash + 15485867 * (right->hash + 141650963 * std::hash<int>{}(op));

        height = 1 + std::max(left->height, right->height);
        return;
    }

    if (isUnary())
    {
        left->rehash();
        hash = left->hash + 256203161 * std::hash<int>{}(op);
        height = 1 + left->height;
        return;
    }

    cout << "ASTNodeInt::rehash():UNSUPPORTED " << getName(op) << endl;
}


void ASTNodeInt::print()
{
    switch (op) {
        case OPREATOR_ID: {
            cout << name;
            break;
        }

        case OPREATOR_NUM: {
            cout << "value(" << val << ")";
            break;
        }

        case OPERATOR_ADD : {
            cout << "+";
            break;
        }
        case OPERATOR_SUB : {
            cout << "-";
            break;
        }
        case OPERATOR_MUL : {
            cout << "*";
            break;
        }
        case OPERATOR_DIV : {
            cout << "/";
            break;
        }

        case OPERATOR_POW : {
            cout << "pow";
            break;
        }
        case OPERATOR_SIN: {
            cout << "sin";
            break;
        }
        case OPERATOR_COS: {
            cout << "cos";
            break;
        }

        default   : {
            cout << " UNSUPPORTED ";
            break;
        }
    }
}

/*
void ASTNodeInt::printRec(int ident, ASTRenderDec identSym)
{


}*/

void ASTNodeInt::getVars(std::vector<std::string> &result)
{
    if (op == OPREATOR_ID) {
        if (std::find(result.begin(), result.end() ,name) == result.end()) {
            result.push_back(name);
        }
    } else {
        //cout << "ZZ ";
        if (left != NULL)
            left ->getVars(result);
        if (right != NULL)
            right->getVars(result);
    }
}

ASTNodeInt *ASTNodeInt::derivative(const std::string &var)
{
    switch (op)
    {
        case OPREATOR_ID:
        {
            if (name == var)
                return new ASTNodeInt(1.0);
            else
                return new ASTNodeInt(0.0);
            break;
        }

        case OPREATOR_NUM:
        {
            return new ASTNodeInt(0.0);
            break;
        }

        case OPERATOR_ADD:
        {
            return new ASTNodeInt(OPERATOR_ADD, left->derivative(var), right->derivative(var));
            break;
        }

        case OPERATOR_SUB:
        {
            return new ASTNodeInt(OPERATOR_SUB, left->derivative(var), right->derivative(var));
            break;
        }

        case OPERATOR_MUL:
        {
            ASTNodeInt *leftD  = left ->derivative(var);
            ASTNodeInt *rightD = right->derivative(var);


            return new ASTNodeInt(OPERATOR_ADD,
                          new ASTNodeInt(OPERATOR_MUL, leftD, right ),
                          new ASTNodeInt(OPERATOR_MUL,  left, rightD)
                   );
            break;
        }

        case OPERATOR_DIV:
        {
            ASTNodeInt *leftD  = left ->derivative(var);
            ASTNodeInt *rightD = right->derivative(var);


            return new ASTNodeInt(OPERATOR_DIV,
                        new ASTNodeInt(OPERATOR_SUB,
                               new ASTNodeInt(OPERATOR_MUL, leftD, right ),
                               new ASTNodeInt(OPERATOR_MUL,  left, rightD)
                        ),
                        new ASTNodeInt(OPERATOR_MUL, right,  right)
                   );
            break;
        }

        case OPERATOR_POW:
        {
            /*So far partial support */
            if (right->op == OPREATOR_NUM) {
                ASTNodeInt *leftD  = left->derivative(var);

                return new ASTNodeInt(OPERATOR_MUL,
                       new ASTNodeInt(right->val),
                       new ASTNodeInt(OPERATOR_MUL,
                               new ASTNodeInt(OPERATOR_POW,
                                    left,
                                    new ASTNodeInt(right->val - 1.0)
                               ),
                               leftD
                           )
                       );
            }
            return new ASTNodeInt("UDEF");
            break;
        }

        default:
            printf(" UNSUPPORTED OPERATOR %d", op);
            return new ASTNodeInt("UDEF");
            break;
    }
}

ASTNodeInt *ASTNodeInt::compute(const std::map<std::string, double> &bind)
{
    // print();
    // cout << endl;
    switch (op) {
        case OPREATOR_ID: {
            auto it = bind.find(name);
            if (it != bind.end())
            {
                return new ASTNodeInt(it->second);
            } else {
                return this;
            }
            break;
        }

        case OPREATOR_NUM: {
            return this;
            break;
        }

        case OPERATOR_ADD : {
            ASTNodeInt *nleft  = left->compute(bind);
            ASTNodeInt *nright = right->compute(bind);

            if (nleft->op == OPREATOR_NUM && nright->op == OPREATOR_NUM)
            {
                return new ASTNodeInt(nleft->val + nright->val);
            }
            if (nleft->op == OPREATOR_NUM && nleft->val == 0.0)
            {
                return nright;
            }
            if (nright->op == OPREATOR_NUM && nright->val == 0.0)
            {
                return nleft;
            }
            return new ASTNodeInt(OPERATOR_ADD, nleft, nright);
            break;
        }
        case OPERATOR_SUB : {
            ASTNodeInt *nleft  = left ->compute(bind);
            ASTNodeInt *nright = right->compute(bind);

            if (nleft->op == OPREATOR_NUM && nright->op == OPREATOR_NUM)
            {
                return new ASTNodeInt(nleft->val - nright->val);
            }
            if (nright->op == OPREATOR_NUM && nright->val == 0.0)
            {
                return nleft;
            }
            return new ASTNodeInt(OPERATOR_SUB, nleft, nright);
            break;
        }
        case OPERATOR_MUL : {
            ASTNodeInt *nleft  = left->compute(bind);
            ASTNodeInt *nright = right->compute(bind);

            if (nleft->op == OPREATOR_NUM && nright->op == OPREATOR_NUM)
            {
                return new ASTNodeInt(nleft->val * nright->val);
            }
            if (nleft->op == OPREATOR_NUM && nleft->val == 0.0)
            {
                return new ASTNodeInt(0.0);
            }
            if (nright->op == OPREATOR_NUM && nright->val == 0.0)
            {
                return new ASTNodeInt(0.0);
            }
            if (nleft->op == OPREATOR_NUM && nleft->val == 1.0)
            {
                return nright;
            }
            if (nright->op == OPREATOR_NUM && nright->val == 1.0)
            {
                return nleft;
            }
            return new ASTNodeInt(OPERATOR_MUL, nleft, nright);
            break;
        }
        case OPERATOR_DIV : {
            ASTNodeInt *nleft  = left->compute(bind);
            ASTNodeInt *nright = right->compute(bind);

            if (nleft->op == OPREATOR_NUM && nright->op == OPREATOR_NUM)
            {
                return new ASTNodeInt(nleft->val / nright->val);
            }
            if (nright->op == OPREATOR_NUM && nright->val == 1.0)
            {
                return nleft;
            }
            if (nleft->op == OPREATOR_NUM && nleft->val == 0.0)
            {
                return new ASTNodeInt(0.0);
            }
            return new ASTNodeInt(OPERATOR_DIV, nleft, nright);
            break;
        }
        case OPERATOR_POW : {
            ASTNodeInt *nleft  = left ->compute(bind);
            ASTNodeInt *nright = right->compute(bind);

            if (nleft->op == OPREATOR_NUM && nright->op == OPREATOR_NUM)
            {
                return new ASTNodeInt(pow(nleft->val, nright->val));
            }
            return new ASTNodeInt(OPERATOR_POW, nleft, nright);
        }
        default   :
            printf("ASTNodeInt::compute(): UNSUPPORTED OP %d - %s\n", op, getName(op));
            return new ASTNodeInt("UDEF");
        break;
    }
}

void ASTNodeInt::deleteSubtree(ASTNodeInt *tree)
{
    tree->deleteChildren();
    delete(tree);
}

void ASTNodeInt::deleteChildren()
{
    if (isBinary())
    {
        left->deleteChildren();
        right->deleteChildren();
        delete left;
        delete right;
        return;
    }

    if (isUnary())
    {
        left->deleteChildren();
        delete left;
    }
}



void ASTContext::clear() {
    for (ASTNodeInt *node : nodes)
    {
        node->markFlag = 0;
    }
}

void ASTContext::mark(ASTNodeInt *node) {
    node->markFlag = 1;

    if (node->isBinary())
    {
        mark(node->left);
        mark(node->right);
        return;
    }

    if (node->isUnary())
    {
        mark(node->left);
        return;
    }
}

void ASTContext::sweep() {
    nodes.erase(
        remove_if(nodes.begin(), nodes.end(),
                  [](ASTNodeInt *node){
            return (node->markFlag == 0);
        }));
}



} //namespace corecvs
