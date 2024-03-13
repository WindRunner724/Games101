#pragma warning(disable: 4244)
#pragma warning(disable: 4305)
#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        //splitMethod = SplitMethod::SAH;
        switch (splitMethod) {
        case SplitMethod::NAIVE: {
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
            break;
        }
        case SplitMethod::SAH: {
            float beginning = objects[0]->getBounds().Centroid()[dim];
            float ending = objects[objects.size() - 1]->getBounds().Centroid()[dim];
            int bucketNum = 12;
            float bucketGap = (ending - beginning) / bucketNum;
            std::vector<Bounds3> surfaceForwardList(bucketNum);
            std::vector<Bounds3> surfaceBackwardList(bucketNum);
            std::vector<int> objForwardNum(bucketNum);
            int objForwardFlag = 0;
            int objBackwardFlag = objects.size() - 1;
            Bounds3 totalBound;
            for (int i = 0; i < objects.size(); ++i)
                totalBound = Union(totalBound, objects[i]->getBounds());

            for (int i = 1; i < bucketNum; i++)
            {
                surfaceForwardList[i] = surfaceForwardList[i - 1];
                objForwardNum[i] = objForwardNum[i - 1];
                float left = objects[objForwardFlag]->getBounds().Centroid()[dim];
                float right = objects[objBackwardFlag]->getBounds().Centroid()[dim];
                float leftEdge = beginning + bucketGap * i;
                float rightEdge = ending - bucketGap * i;
                while (objects[objForwardFlag]->getBounds().Centroid()[dim] <= beginning + bucketGap * i) {
                    surfaceForwardList[i] = Union(surfaceForwardList[i], objects[objForwardFlag++]->getBounds());
                    objForwardNum[i]++;
                }
                while (objects[objBackwardFlag]->getBounds().Centroid()[dim] > ending - bucketGap * i) {
                    surfaceBackwardList[i] = Union(surfaceBackwardList[i], objects[objBackwardFlag--]->getBounds());
                }
            }
            double totalArea = totalBound.SurfaceArea();
            double minCost = std::numeric_limits<double>::max();
            int optBucketId;
            for (int i = 1; i < bucketNum; i++)
            {
                double cost = surfaceForwardList[i].SurfaceArea() / totalArea * objForwardNum[i]
                    + surfaceBackwardList[objects.size() - i].SurfaceArea() / totalArea * (objects.size() - objForwardNum[i]);
                if (cost < minCost)
                {
                    minCost = cost;
                    optBucketId = i;
                }
            }

            surfaceForwardList.clear();
            surfaceForwardList.shrink_to_fit();
            surfaceBackwardList.clear();
            surfaceBackwardList.shrink_to_fit();
            auto leftshapes = std::vector<Object*>(objects.begin(), objects.begin() + objForwardNum[optBucketId]);
            auto rightshapes = std::vector<Object*>(objects.begin() + objForwardNum[optBucketId], objects.end());

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = std::move(totalBound);
            break;
        }
        default:
            break;
        }

        //auto beginning = objects.begin();
        //auto middling = objects.begin() + (objects.size() / 2);
        //auto ending = objects.end();

        //auto leftshapes = std::vector<Object*>(beginning, middling);
        //auto rightshapes = std::vector<Object*>(middling, ending);

        //assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        //node->left = recursiveBuild(leftshapes);
        //node->right = recursiveBuild(rightshapes);

        //node->bounds = Union(node->left->bounds, node->right->bounds);
        //node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (root && root->bounds.IntersectP(ray, ray.direction_inv, { int(ray.direction.x > 0),int(ray.direction.y > 0),int(ray.direction.z > 0) }))
        isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection lisect, risect;
    std::array<int, 3> dirIsNeg = { int(ray.direction.x >= 0),int(ray.direction.y >= 0),int(ray.direction.z >= 0) };

    if (node->left && node->left->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        lisect = getIntersection(node->left, ray);
    if (node->right && node->right->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        risect = getIntersection(node->right, ray);
    if (lisect.happened && risect.happened)
        return (lisect.distance < risect.distance) ? lisect : risect;
    else if (lisect.happened) return lisect;
    else if (risect.happened) return risect;

    return (node->object) ? node->object->getIntersection(ray) : Intersection();
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}