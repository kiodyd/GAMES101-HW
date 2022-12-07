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
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        std::vector<Object*> leftshapes, rightshapes;
        switch (splitMethod) {
            case SplitMethod::NAIVE:
            {
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

                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() / 2);
                auto ending = objects.end();

                leftshapes = std::vector<Object*>(beginning, middling);
                rightshapes = std::vector<Object*>(middling, ending);

                break;
            }
            case SplitMethod::SAH:
            {
                // 桶的数量
                constexpr int bucketCount = 32;
                // 记录每一维桶的分配
                std::vector<std::vector<std::vector<int>>> bucketIncludeIndex;
                float minCost = std::numeric_limits<float>::infinity();
                int minCostCoor;
                int minCostSplitBid;
                for (int i = 0; i < 3; ++i) {
                    std::vector<Bounds3> buckets;
                    std::vector<std::vector<int>> indexList;
                    // 初始化桶
                    for (int bid = 0; bid < bucketCount; ++bid) {
                        buckets.emplace_back();
                        indexList.emplace_back();
                    }
                    // 分配桶
                    for (int oid = 0; oid < objects.size(); ++oid) {
                        Vector3f offset = centroidBounds.Offset(objects[oid]->getBounds().Centroid());

                        // fix 不知道为什么arm架构[]操作符报错
                        double axisValue;
                        switch (i) {
                            case 0: axisValue = offset.x;break;
                            case 1: axisValue = offset.y;break;
                            case 2: axisValue = offset.z;break;
                        }

                        int bid = (int)(bucketCount * axisValue);
                        if (bid >= bucketCount)
                            bid = bucketCount - 1;
                        buckets[bid] = Union(buckets[bid],objects[oid]->getBounds().Centroid());
                        indexList[bid].push_back(oid);
                    }

                    bucketIncludeIndex.push_back(indexList);

                    // 使用启发公式计算最佳分配
                    for (int bid = 1; bid < bucketCount; ++bid) {
                        Bounds3 leftBounds, rightBounds;
                        int leftCount = 0, rightCount = 0;

                        for (int left = 0; left < bid; ++left) {
                            leftBounds = Union(leftBounds, buckets[left]);
                            leftCount += indexList[left].size();
                        }
                        for (int right = bid; right < bucketCount; ++right) {
                            rightBounds = Union(rightBounds, buckets[right]);
                            rightCount += indexList[right].size();
                        }

                        float cost =
                                (leftCount * leftBounds.SurfaceArea() + rightCount * rightBounds.SurfaceArea()) / centroidBounds.SurfaceArea();

                        if (cost < minCost) {
                            minCostCoor = i;
                            minCostSplitBid = bid;
                            minCost = cost;
                        }
                    }
                }

                // 将最小花费分配结果写入
                for (int i = 0; i < bucketCount; ++i) {
                    for (int j : bucketIncludeIndex[minCostCoor][i]) {
                        if (i < minCostSplitBid)
                            leftshapes.push_back(objects[j]);
                        else
                            rightshapes.push_back(objects[j]);
                    }
                }
                break;
            }
        }

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;

    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = ray.direction.x > 0 ? 1 : 0;
    dirIsNeg[1] = ray.direction.y > 0 ? 1 : 0;
    dirIsNeg[2] = ray.direction.z > 0 ? 1 : 0;

    if (node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        if (node->left == nullptr && node->right == nullptr)
        {
            // object即可能是mesh物体，也可能是三角面
            inter = node->object->getIntersection(ray);
        }
        else
        {
            auto left = getIntersection(node->left, ray);
            auto right = getIntersection(node->right, ray);
            inter = left.distance < right.distance ? left : right;
        }
    }
    return inter;
}