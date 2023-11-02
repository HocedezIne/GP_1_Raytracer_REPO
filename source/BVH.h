#pragma once

#include <cassert>

#include "Math.h"
#include "vector"
#include "DataTypes.h"
#include <algorithm>

// used chatgtp and https://fileadmin.cs.lth.se/cs/Education/EDAN30/lectures/S2-bvh.pdf to understand this concept and write the code below

namespace dae
{
	struct BVHNode
	{
		Vector3 minAABB{};
		Vector3 maxAABB{};
		BVHNode* left{};
		BVHNode* right{};
		GeometryObj* object{};
	};

	class BVH
	{
	public:
		BVH(std::vector<GeometryObj*>& objects)
		{
			m_Root = BuildBVHTree(objects, 0, int(objects.size() - 1));
		}

		BVHNode* BuildBVHTree(std::vector<GeometryObj*>& objects, int startIdx, int endIdx)
		{
			if (endIdx < 0) return nullptr;

			if (startIdx == endIdx) // only one obj to be stored
			{
				BVHNode* leaf = new BVHNode;
				leaf->object = objects[startIdx];
				leaf->object->GetBoundingBox(leaf->minAABB, leaf->maxAABB);
				return leaf;
			}
			else // recursively build tree
			{
				BVHNode* node = new BVHNode;
				GetSharedBoundingBox(objects, startIdx, endIdx, node); // find boundingbox of all objects at this level

				// splitting critera is longest edge -> determine longest edge
				const float x = node->maxAABB.x - node->minAABB.x;
				const float y = node->maxAABB.y - node->minAABB.y;
				const float z = node->maxAABB.z - node->minAABB.z;

				// sorting baed on longest edge
				if (x > y && x > z) // x axis longest edge
				{
					std::sort(objects.begin() + startIdx, objects.begin() + endIdx, [](GeometryObj* a, GeometryObj* b) {
						return a->GetOrigin().x < b->GetOrigin().x;
					});
				}
				else if (y > z) // y axis longest edge
				{
					std::sort(objects.begin() + startIdx, objects.begin() + endIdx, [](GeometryObj* a, GeometryObj* b) {
						return a->GetOrigin().y < b->GetOrigin().y;
					});
				}
				else // z axis longest edge
				{
					std::sort(objects.begin() + startIdx, objects.begin() + endIdx, [](GeometryObj* a, GeometryObj* b) {
						return a->GetOrigin().z < b->GetOrigin().z;
					});
				}

				// split up objects and recall build function
				int mid = (startIdx + endIdx) / 2;
				node->left = BuildBVHTree(objects, startIdx, mid);
				node->right = BuildBVHTree(objects, mid + 1, endIdx);

				return node;
			}
		}

	private:
		void GetSharedBoundingBox(std::vector<GeometryObj*>& objects, int startIdx, int endIdx, BVHNode*& node)
		{
			objects[startIdx]->GetBoundingBox(node->minAABB, node->maxAABB);
			for (int idx = startIdx+1; idx < endIdx; idx++)
			{
				Vector3 newMin, newMax;
				objects[idx]->GetBoundingBox(newMin, newMax);
				node->minAABB = Vector3::Min(node->minAABB, newMin);
				node->maxAABB = Vector3::Max(node->maxAABB, newMax);
			}
		}

		BVHNode* m_Root;
	};
}