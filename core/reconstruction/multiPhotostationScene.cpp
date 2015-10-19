#include "multiPhotostationScene.h"

#include "multicameraTriangulator.h"
#include "bufferReaderProvider.h"
#include "calibrationHelpers.h"

corecvs::Vector2dd MultiPhotostationScene::computeReprojectionError (int ps, int cam, const corecvs::Vector3dd &point, const corecvs::Vector2dd &expected) const
{
    return project(ps, cam, point) - expected;
}
void   MultiPhotostationScene::computeReprojectionErrors(int ps, int cam, std::vector<double> &errors) const
{
    errors.clear();
    for (auto& p: pointObservations)
        for (auto& c: p.projections)
        {
            if (ps == c.photostationId && cam == c.cameraId)
            {
                auto e = computeReprojectionError(ps, cam, p.worldPoint, c.projection);
                errors.push_back(e[0]);
                errors.push_back(e[1]);
            }
        }
}
void MultiPhotostationScene::computeReprojectionErrors(std::vector<double> &errors) const
{
    int M = (int)photostations.size();
    errors.clear();
    for (int i = 0; i < M; ++i)
    {
        int N = (int)photostations[i].cameras.size();
        for (int j = 0; j < N; ++j)
        {
            std::vector<double> err;
            computeReprojectionErrors(i, j, err);
            errors.insert(errors.end(), err.begin(), err.end());
        }
    }
}
corecvs::Vector2dd MultiPhotostationScene::project(int ps, int cam, const corecvs::Vector3dd &point) const
{
    return photostations[ps].project(point, cam);
}

corecvs::Vector3dd MultiPhotostationScene::BackProject(const std::vector<std::pair<corecvs::Matrix44, corecvs::Vector2dd>> &points)
{
    corecvs::MulticameraTriangulator mct;
    for (auto& p: points)
    {
        mct.addCamera(p.first, p.second);
    }
    return /*mct.triangulateLM(*/mct.triangulate();//);
}

corecvs::Vector3dd MultiPhotostationScene::backProject(const std::vector<std::pair<std::pair<int, int>, corecvs::Vector2dd>> &points) const
{
    std::vector<std::pair<corecvs::Matrix44, corecvs::Vector2dd>> pairs;
    for (auto &pd: points)
    {
        CORE_ASSERT_TRUE_S(pd.first.first < photostations.size());
        CORE_ASSERT_TRUE_S(pd.first.second < photostations[pd.first.first].cameras.size());
        auto matrix = photostations[pd.first.first].getKMatrix(pd.first.second);
        auto point = pd.second;
        pairs.emplace_back(matrix, point);
    }
    return BackProject(pairs);
}
corecvs::Vector3dd MultiPhotostationScene::backProject(const std::vector<PointProjection> &projections) const
{
    std::vector<std::pair<std::pair<int, int>, corecvs::Vector2dd>> points;
    for (auto& proj: projections)
    {
        auto& kp = cameraObservations[proj.photostationId][proj.cameraId].keyPoints[proj.featureId];
        points.emplace_back(std::make_pair(proj.photostationId, proj.cameraId), corecvs::Vector2dd(kp.x, kp.y));
    }
    CORE_ASSERT_TRUE_S(points.size() > 0);
    return backProject(points);
}

void MultiPhotostationScene::ParallelUpdater::operator() (const corecvs::BlockedRange<int> &r) const
{
    for (int i = r.begin(); i < r.end(); ++i)
    {
        scene->pointObservations[i].worldPoint = scene->backProject(scene->pointObservations[i].projections);
    }
}

void MultiPhotostationScene::updateBackProjections()
{
    corecvs::parallelable_for(0, (int)pointObservations.size(), ParallelUpdater(this));
}

int MultiPhotostationScene::getPhotostationCount() const
{
    CORE_ASSERT_TRUE_S(photostations.size() == cameraObservations.size());
    for (size_t i = 0; i < photostations.size(); ++i) {
        CORE_ASSERT_TRUE_S(photostations[i].cameras.size() == cameraObservations[i].size());
    }
    return (int)photostations.size();
}

void MultiPhotostationScene::ParallelColoriser::operator() (const corecvs::BlockedRange<int> &range) const
{
    auto& scene = *scene_;
    auto& colors = *colors_;

    int cc = 0;
    int tc = range.end() - range.begin();
    for (int i = range.begin(); i < range.end(); ++i)
    {
        int r, g, b, c;
        r = g = b = c = 0;
        cc++;
        auto& pt = scene.pointObservations[i];
        for (auto &p: pt.projections)
        {
            auto kp = scene.cameraObservations[p.photostationId][p.cameraId].keyPoints[p.featureId];
            auto fname = scene.cameraObservations[p.photostationId][p.cameraId].undistortedFileName;
            corecvs::RGB24Buffer src = BufferReaderProvider::readRgb(fname);
            auto cc = src.element((int)kp.y, (int)kp.x);
            r += cc.r();
            g += cc.g();
            b += cc.b();
            c++;
            //            break;
        }
        r /= c;
        g /= c;
        b /= c;
        colors[i] = corecvs::RGBColor(r, g, b);
        std::cout << cc << "/" << tc << " processed (" << ((double)cc)/((double)tc)*100.0 << ")" << std::endl;
    }
}

void MultiPhotostationScene::drawPly(corecvs::Mesh3D &mesh)
{
    for (auto& ps : photostations) {
        corecvs::CalibrationHelpers().drawPly(mesh, ps, 1000);
    }
    int obscnt = 0;
    int NN = (int)pointObservations.size();
    std::vector<corecvs::RGBColor> colors(NN);
    corecvs::parallelable_for(0, NN, NN / 8, ParallelColoriser(&colors, this));

    int id = 0;
    for (auto& pt: pointObservations)
    {
        mesh.currentColor = colors[id++];
        mesh.addPoint(pt.worldPoint);
        std::cout << obscnt << "/" << pointObservations.size() << " processed (" << ((double)obscnt)/((double)pointObservations.size())*100.0 << ")" << std::endl;
    }
}

corecvs::Mesh3D MultiPhotostationScene::drawPly()
{
    corecvs::Mesh3D mesh;
    mesh.switchColor(true);
    drawPly(mesh);
    return mesh;
}

