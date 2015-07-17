#include "selectableGeometryFeatures.h"

namespace corecvs {

SelectableGeometryFeatures::Vertex::Vertex(const Vector2dd &_position) :
    mSelected(false),
    position(_position),
    weight(-1.0),
    ownerPath(NULL)
{}

bool SelectableGeometryFeatures::Vertex::isSelected()
{
    return mSelected;
}


void SelectableGeometryFeatures::addSelection(SelectableGeometryFeatures::Vertex *vertex)
{
    mSelectedPoints.push_back(vertex);
    vertex->mSelected = true;
}

void SelectableGeometryFeatures::removeSelection(SelectableGeometryFeatures::Vertex *vertex)
{
    vector<Vertex*>::iterator it = std::remove(mSelectedPoints.begin(), mSelectedPoints.end(), vertex);
    mSelectedPoints.erase( it, mSelectedPoints.end() );
    vertex->mSelected = false;
}

void SelectableGeometryFeatures::deselectAllPoints()
{
    for (unsigned i = 0; i < mSelectedPoints.size(); i++)
    {
        mSelectedPoints[i]->mSelected = false;
    }
    mSelectedPoints.clear();
}

SelectableGeometryFeatures::Vertex *SelectableGeometryFeatures::findClosest(const Vector2dd &position)
{
    double minDist = numeric_limits<double>::max();
    Vertex *result =  NULL;
    for (unsigned i = 0; i < mPoints.size(); i++)
    {
        double dist = (position - mPoints[i]->position).l2Metric();
        if (dist < minDist)
        {
            minDist = dist;
            result = mPoints[i];
        }
    }
    return result;
}

SelectableGeometryFeatures::VertexPath*  SelectableGeometryFeatures::appendNewPath()
{
     mPaths.push_back(new VertexPath());
     return mPaths.back();
}

void SelectableGeometryFeatures::deletePath(SelectableGeometryFeatures::VertexPath *path)
{
    if (path->isSelected())
    {
        removeSelection(path);
    }
    for (unsigned i = 0; i < path->vertexes.size(); i++)
    {
        path->vertexes[i]->ownerPath = NULL;
    }
    vector<VertexPath *>::iterator it = std::remove(mPaths.begin(), mPaths.end(), path);
    mPaths.erase( it, mPaths.end() );
    delete path;
}

SelectableGeometryFeatures::Vertex* SelectableGeometryFeatures::appendNewVertex(const Vector2dd &point)
{
    mPoints.push_back(new Vertex(point));
    mPoints.back()->ownerPath = NULL;
    return mPoints.back();
}


void SelectableGeometryFeatures::deselectAllPath()
{
    for (unsigned i = 0; i < mSelectedPaths.size(); i++)
    {
        mSelectedPaths[i]->mSelected = false;
    }
    mSelectedPaths.clear();
}

void SelectableGeometryFeatures::addVertexToPath(SelectableGeometryFeatures::Vertex *vertex, SelectableGeometryFeatures::VertexPath *path)
{
    path->vertexes.push_back(vertex);
    vertex->ownerPath = path;
}

void SelectableGeometryFeatures::removeVertexFromPath(SelectableGeometryFeatures::Vertex *vertex, bool purgeEmptyPath)
{
    VertexPath *ownerPath = vertex->ownerPath;

    if (ownerPath == NULL)
    {
        return;
    }

    vector<Vertex*>::iterator it = std::remove(ownerPath->vertexes.begin(), mSelectedPoints.end(), vertex);
    ownerPath->vertexes.erase( it, mSelectedPoints.end() );

    if (ownerPath->isEmpty() && purgeEmptyPath)
    {
        deletePath(ownerPath);
    }
}

void SelectableGeometryFeatures::addSelection(SelectableGeometryFeatures::VertexPath *path)
{
    mSelectedPaths.push_back(path);
    path->mSelected = true;
}

void SelectableGeometryFeatures::removeSelection(SelectableGeometryFeatures::VertexPath *path)
{
    vector<VertexPath*>::iterator it = std::remove(mSelectedPaths.begin(), mSelectedPaths.end(), path);
    mSelectedPaths.erase( it, mSelectedPaths.end() );
    path->mSelected = false;
}

void SelectableGeometryFeatures::deleteVertex(Vertex *vertex)
{
    VertexPath *ownerPath = vertex->ownerPath;

    removeSelection(vertex);

    if (ownerPath != NULL) {
        removeVertexFromPath(vertex);
    }

    vector<Vertex *>::iterator it = std::remove(mPoints.begin(), mPoints.end(), vertex);
    mPoints.erase( it, mPoints.end() );
    delete vertex;
}

/*
void SelectableGeometryFeatures::deleteVertex(int id)
{
    VertexPath *ownerPath = mPoints[id].ownerPath;
    do
    {
        if (ownerPath == NULL)
        {
            break;
        }

        ownerPath->vertexes.removeAll(&(mPoints[id]));
        if (!ownerPath->vertexes.isEmpty())
        {
            break;
        }

        mSelectedPaths.removeAll(ownerPath);
        for (int i = 0; i < mPaths.size(); i++)
        {
            if (&mPaths[i] == ownerPath)
            {
                mPaths.removeAt(i);
                break;
            }
        }
    } while (false);
    mPoints.removeAt(id);
    mUi->widget->update();
}*/

void SelectableGeometryFeatures::deleteVertex(const Vector2dd &point)
{
    vector<Vertex*> toDelete;

    for (unsigned i = 0; i < mPoints.size(); i++)
    {
        if (mPoints[i]->position == point)
        {
            toDelete.push_back(mPoints[i]);
        }
    }

    for (unsigned i = 0; i < toDelete.size(); i++)
    {       
        deleteVertex(toDelete[i]);
    }
}


void SelectableGeometryFeatures::clearAll()
{
    mSelectedPaths.clear();
    mSelectedPoints.clear();

    mPaths.clear();
    mPoints.clear();
}

void SelectableGeometryFeatures::deselectAll()
{
    deselectAllPath();
    deselectAllPoints();
}

bool SelectableGeometryFeatures::hasSinglePointsSelected()
{
    int i = 0;
    for (; i < mSelectedPoints.size(); i++)
    {
        if (!mSelectedPoints[i]->isInPath()) {
            return true;
        }
    }

    return false;
}

SelectableGeometryFeatures::SelectableGeometryFeatures()
{
}

SelectableGeometryFeatures::~SelectableGeometryFeatures()
{
    clearAll();
}

vector<vector<Vector2dd> > SelectableGeometryFeatures::getLines()
{
    vector<vector<Vector2dd> > toReturn;
    for (unsigned i = 0; i < mPaths.size(); i++)
    {
        toReturn.resize(toReturn.size() + 1);
        for (unsigned j = 0; j < mPaths[i]->vertexes.size(); j++)
        {
            toReturn.back().push_back(mPaths[i]->vertexes.operator[](j)->position);
        }
    }
    return toReturn;
}

bool SelectableGeometryFeatures::VertexPath::isSelected()
{
    return mSelected;
}

bool SelectableGeometryFeatures::VertexPath::isEmpty()
{
    return vertexes.empty();
}


}
