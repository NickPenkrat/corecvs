#include <set>

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
    delete_safe (vertex);
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

void SelectableGeometryFeatures::addPathFrom(const SelectableGeometryFeatures &other)
{
#if 0
    int index = mPoints.size();

    for (int i = 0; i < other.mPoints.size(); i++)
    {
        appendNewVertex(other.mPoints[i]->position);
    }

    /* We match the vertexes by offset... not the most clean way */
    for (int i = 0; i < other.mPaths.size(); i++)
    {
        VertexPath *path = appendNewPath();
        VertexPath *otherpath = other.mPaths[i];
        for (unsigned j = 0; j < otherpath->vertexes.size(); j++)
        {
            Vertex *toCopy = otherpath->vertexes[j];
            /* TODO: oh my god! O(n^3)??? really */


            int index = toCopy - &(other.mPoints[0]);
            Vertex *copy = mPoints[beforeAdd + index];
            addVertexToPath(copy, path);
        }
//        appendNewVertex(other.mPoints[i]->position);
    }
#endif

    for (unsigned i = 0; i < other.mPaths.size(); i++)
    {
        VertexPath *path = appendNewPath();
        VertexPath *otherpath = other.mPaths[i];
        for (unsigned j = 0; j < otherpath->vertexes.size(); j++)
        {
            Vertex *toCopy = otherpath->vertexes[j];
            addVertexToPath(appendNewVertex(toCopy->position), path);
        }
    }

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

void SelectableGeometryFeatures::print()
{
    cout << "Size is:" << mPaths.size() << std::endl;
    for (unsigned i = 0; i < mPaths.size(); i++)
    {
        cout << "[" << mPaths[i]->vertexes.size() << " :";
        for (unsigned j = 0; j < mPaths[i]->vertexes.size(); j++)
        {
            cout  << (j == 0 ? "" : ",") << mPaths[i]->vertexes.operator[](j)->position;
        }
        cout << "]" << std::endl;
    }
}

bool SelectableGeometryFeatures::VertexPath::isSelected()
{
    return mSelected;
}

bool SelectableGeometryFeatures::VertexPath::isEmpty()
{
    return vertexes.empty();
}

int SelectableGeometryFeatures::VertexPath::length()
{
    return vertexes.size();
}

SelectableGeometryFeatures::addAllLinesFromObservationList(const ObservationList &list)
{
    const double LINETOLERANCE = 1e-7;
    int N = list.size();
    std::vector<int> used(N * N);
    std::vector<std::vector<int>> lines;
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            if (used[i * N + j] || used[j * N + i])
                continue;
            used[i * N + j] = used[j * N + i] = 1;
            corecvs::Vector3dd A, B, C, d, D;
            A = list[i].point;
            B = list[j].point;
            d = (B - A).normalised();

            std::vector<int> line = {i, j};
            for (int k = 0; k < N; ++k)
            {
                if (k == i || k == j)
                    continue;
                C = list[k].point;
                D = C - A;
                double diff = !(D - (D & d) * d);

                if (diff < LINETOLERANCE)
                {
                    used[i * N + k] = used[k * N + i] = used[j * N + k] = used[k * N + j] = 1;
                    line.push_back(k);
                }
            }

            if (line.size() >= 3)
            {
                std::sort(line.begin(), line.end());
                lines.push_back(line);
            }
        }
    }

    int idx = 0;
    for (auto& line: lines)
    {
        bool unique = true;
        std::set<int> cset(line.begin(), line.end());
        for (int i = 0; i < idx; ++i)
        {
            int common = 0;
            for (auto& id: lines[i])
                if (cset.count(id))
                    common++;
            if (common >= 2)
            {
                unique = false;
                break;
            }
        }

        if (unique)
            lines[idx++] = line;
    }
    lines.resize(idx);

    for (auto& line: lines)
    {
        auto* path = appendNewPath();
        for (auto id: line)
            addVertexToPath(appendNewVertex(list[id].projection), path);
    }
}
}
