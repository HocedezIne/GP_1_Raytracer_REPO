#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 originToOrigin{ sphere.origin - ray.origin };
			const float originToOriginDot{ Vector3::Dot(originToOrigin, ray.direction) };

			const float discriminant = Square(sphere.radius) - Vector3::Dot(originToOrigin, originToOrigin) + Square(originToOriginDot);

			if (discriminant <= 0) return false;

			const float tHC{ sqrt(discriminant) };
			const float t0{ originToOriginDot - tHC };
			const float t1{ originToOriginDot + tHC };

			if (t0 < ray.min)
			{
				if (t1 < ray.min || t1 > ray.max) return false;
				hitRecord.t = t1;
			}
			else if (t0 > ray.max) return false;
			else hitRecord.t = t0;

			if (ignoreHitRecord) return true;

			hitRecord.didHit = true;
			hitRecord.materialIndex = sphere.materialIndex;
			hitRecord.origin = ray.origin + hitRecord.t * ray.direction;
			hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
			return true;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const float t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / Vector3::Dot(ray.direction, plane.normal);

			if (t > ray.min && t < ray.max)
			{
				if (ignoreHitRecord) return true;
				hitRecord.didHit = true;
				hitRecord.materialIndex = plane.materialIndex;
				hitRecord.t = t;
				hitRecord.origin = ray.origin + t * ray.direction;
				hitRecord.normal = plane.normal;
				return true;
			}

			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			// Müller-Trombore - based on https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
			// triangle edges in v0
			const Vector3 e1v0{ triangle.v1 - triangle.v0 };
			const Vector3 e2v0{ triangle.v2 - triangle.v0 };

			const Vector3 n{ Vector3::Cross(e1v0, e2v0) };
			const float normalRayDot{ Vector3::Dot(n, ray.direction) };
			if (AreEqual(normalRayDot, 0)) return false; // ray is parrallel to triangle

			// culling is different for shadows
			if (!ignoreHitRecord)
			{
				if ((normalRayDot > 0 && triangle.cullMode == TriangleCullMode::BackFaceCulling) ||
					(normalRayDot < 0 && triangle.cullMode == TriangleCullMode::FrontFaceCulling)) return false;
			}
			else
			{
				if ((normalRayDot < 0 && triangle.cullMode == TriangleCullMode::BackFaceCulling) ||
					(normalRayDot > 0 && triangle.cullMode == TriangleCullMode::FrontFaceCulling)) return false;
			}

			// calc determinant
			const Vector3 pvec{ Vector3::Cross(ray.direction, e2v0) };
			const float determinant{ Vector3::Dot(e1v0, pvec) };

			if (abs(determinant) < 1e-6f) return false;

			const float inverseDeterminant{ 1 / determinant };

			const Vector3 tvec{ ray.origin - triangle.v0 };
			const float u{ Vector3::Dot(tvec, pvec) * inverseDeterminant };
			if (u < 0 || u > 1) return false;

			const Vector3 qvec{ Vector3::Cross(tvec, e1v0) };
			const float v{ Vector3::Dot(ray.direction, qvec) * inverseDeterminant };
			if (v < 0 || u + v > 1) return false;

			const float t{ Vector3::Dot(e2v0, qvec) * inverseDeterminant };
	
			// check if t in limits
			if (t < ray.min || t > ray.max) return false;

			if (ignoreHitRecord)
			{
				hitRecord.didHit = true;
				hitRecord.t = 0;
				return true;
			}

			//hit record
			hitRecord.didHit = true;
			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.normal = n.Normalized();
			hitRecord.t = t;
			hitRecord.origin = ray.origin + t * ray.direction;

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest(Vector3 minAABB, Vector3 maxAABB, const Ray& ray)
		{
			const float tx1{ (minAABB.x - ray.origin.x) / ray.direction.x };
			const float tx2{ (maxAABB.x - ray.origin.x) / ray.direction.x };

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			const float ty1{ (minAABB.y - ray.origin.y) / ray.direction.y };
			const float ty2{ (maxAABB.y - ray.origin.y) / ray.direction.y };

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			const float tz1{ (minAABB.z - ray.origin.z) / ray.direction.z };
			const float tz2{ (maxAABB.z - ray.origin.z) / ray.direction.z };

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			// slabtest
			if (!SlabTest(mesh.transformedMinAABB, mesh.transformedMaxAABB, ray)) return false;

			Ray workingRay = ray;

			for (int i{}; i < mesh.indices.size(); i+=3)
			{
				Triangle triangle{ mesh.transformedPositions[mesh.indices[i]], mesh.transformedPositions[mesh.indices[i + 1]], mesh.transformedPositions[mesh.indices[i + 2]], Vector3{} };
				triangle.cullMode = mesh.cullMode;
				triangle.materialIndex = mesh.materialIndex;

				if (HitTest_Triangle(triangle, workingRay, hitRecord, ignoreHitRecord))
				{
					workingRay.max = hitRecord.t;
				}
			}
			
			if (!hitRecord.didHit) return false;

			return true;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			switch (light.type)
			{
			case LightType::Point:
				return { light.origin - origin };
			case LightType::Directional:
				return {light.direction};
			default:
				return{};
			}
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			const Vector3 radius{ light.origin - target };

			switch (light.type)
			{
			case LightType::Point:
				return { light.color * (light.intensity / Vector3::Dot(radius, radius)) };
			case LightType::Directional:
				return { light.color * light.intensity };
			default:
				return{};
			}
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}