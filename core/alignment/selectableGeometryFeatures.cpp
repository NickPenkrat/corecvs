#include "selectableGeometryFeatures.h"

namespace corecvs {

SelectableGeometryFeatures::Vertex::Vertex(const Vector2dd &_position) :
    isSelected(false),
    position(_position),
    weight(-1.0),
    ownerPath(NULL)
{}


void SelectableGeometryFeatures::addSelection(SelectableGeometryFeatures::Vertex &vertex)
{
    mSelectedPoints.push_back(&vertex);
    vertex.isSelected = true;
}

void SelectableGeometryFeatures::removeSelection(SelectableGeometryFeatures::Vertex &vertex)
{
    vector<Vertex*>::iterator it = std::remove(mSelectedPoints.begin(), mSelectedPoints.end(), &vertex);
    mSelectedPoints.erase( it, mSelectedPoints.end() );
    vertex.isSelected = false;
}

void SelectableGeometryFeatures::deselectAllPoints()
{
    for (unsigned i = 0; i < mSelectedPoints.size(); i++)
    {
        mSelectedPoints[i]->isSelected = false;
    }
    mSelectedPoints.clear();
}

SelectableGeometryFeatures::Vertex *SelectableGeometryFeatures::findClosest(const Vector2dd &position)
{
    double minDist = numeric_limits<double>::max();
    Vertex *result =  NULL;
    for (unsigned i = 0; i < mPoints.size(); i++)
    {
        double dist = (position - mPoints[i].position).l2Metric();
        if (dist < minDist)
        {
            minDist = dist;
            result = &mPoints[i];
        }
    }
    return result;
}

void SelectableGeometryFeatures::appendPath()
{
     mPaths.push_back(VertexPath());
}

void SelectableGeometryFeatures::addVertex(const SelectableGeometryFeatures::Vertex &vertex)
{
    mPoints.push_back(vertex);
    mPoints.back().ownerPath = NULL;
}

void SelectableGeometryFeatures::addVertex(const Vector2dd &point)
{
    addVertex(Vertex(point));
}


void SelectableGeometryFeatures::deselectAllPath()
{
    for (unsigned i = 0; i < mSelectedPaths.size(); i++)
    {
        mSelectedPaths[i]->isSelected = false;
    }
    mSelectedPaths.clear();
}

void SelectableGeometryFeatures::addVertexToPath(SelectableGeometryFeatures::Vertex *vertex, SelectableGeometryFeatures::VertexPath *path)
{
    path->vertexes.push_back(vertex);
    vertex->ownerPath = path;
}

void SelectableGeometryFeatures::addSelection(SelectableGeometryFeatures::VertexPath &path)
{
    mSelectedPaths.push_back(&path);
    path.isSelected = true;
}

void SelectableGeometryFeatures::deleteVertex(Vertex *vertex)
{
    for (int i = 0; i < mPoints.size(); i++)
    {
        if (&mPoints[i] == vertex)
        {
            deleteVertex(i);
            return;
        }
    }
}


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
}

void SelectableGeometryFeatures::deleteVertex(const Vector2dd &point)
{
    vector<Vertex*> toDelete;

    for (int i = 0; i < mPoints.size(); i++)
    {
        if (mPoints[i].position == point)
        {
            toDelete.push_back(&mPoints[i]);
        }
    }

    for (int i = 0; i < toDelete.size(); i++)
    {
        Vertex *vertex = toDelete[i];
        removeSelection(*vertex);
        deleteVertex(vertex);
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


}
