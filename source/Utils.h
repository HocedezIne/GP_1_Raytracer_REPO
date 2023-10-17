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
			if (ignoreHitRecord) return false;

			const Vector3 originToSphere{ sphere.origin - ray.origin };
			const float originToSphereDot{ Vector3::Dot(originToSphere, ray.direction) };

			const float discriminant = sphere.radius * sphere.radius - Vector3::Dot(originToSphere, originToSphere) + originToSphereDot * originToSphereDot;

			if (discriminant <= 0) return false;

			const float tHC{ sqrtf(discriminant) };

			const float t0{ originToSphereDot - tHC};
			const float t1{ originToSphereDot + tHC};
			if (t0 < ray.min || t1 > ray.max) return false;

			hitRecord.didHit = true;
			hitRecord.materialIndex = sphere.materialIndex;
			hitRecord.t = t0;
			hitRecord.origin = ray.origin + t0 * ray.direction;
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
			if (ignoreHitRecord) return false;

			const float t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / Vector3::Dot(ray.direction, plane.normal);

			if (t > ray.min && t < ray.max)
			{
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
			if (ignoreHitRecord) return false;

			// normal vs raydirection check
			const Vector3 a{ triangle.v1 - triangle.v0 };
			const Vector3 b{ triangle.v2 - triangle.v1 };
			const Vector3 triangleNormal{ Vector3::Cross(a,b) };

			const float normalRayDot{ Vector3::Dot(triangleNormal, ray.direction) };
			if (AreEqual(normalRayDot, 0)) return false; // ray is parrallel to triangle

			// cull mode check
			if ( (normalRayDot > 0 && triangle.cullMode == TriangleCullMode::BackFaceCulling ) ||
				 (normalRayDot < 0 && triangle.cullMode == TriangleCullMode::FrontFaceCulling) )
			{
				return false;
			}

			// ray-plane test + t range check
			const Vector3 L{ triangle.v0 - ray.origin };

			const float t{ Vector3::Dot(L, triangleNormal) / Vector3::Dot(ray.direction, triangleNormal) };
			if (t < ray.min || t > ray.max) return false;

			// check if hitpoint inside triangle
			const Vector3 P{ ray.origin + ray.direction * t };

			const std::vector<const Vector3*> triangleVerts = { &triangle.v0, &triangle.v1, &triangle.v2 };

			for (int i = 0; i < 3; i++)
			{
				const Vector3 e{ *triangleVerts[(i+1)%3] - *triangleVerts[i]};
				const Vector3 p{ P - *triangleVerts[i] };
				if (Vector3::Dot(Vector3::Cross(e, p), triangleNormal) < 0) return false;
			}

			//hit record
			hitRecord.didHit = true;
			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.normal = triangleNormal;
			hitRecord.origin = P;
			hitRecord.t = t;

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			if (ignoreHitRecord) return false;

			Ray workingRay = ray;
			for (int i{}; i < mesh.indices.size(); i+=3)
			{
				Triangle triangle{ mesh.positions[mesh.indices[i]], mesh.positions[mesh.indices[i + 1]], mesh.positions[mesh.indices[i + 2]] };
				triangle.cullMode = mesh.cullMode;
				triangle.materialIndex = mesh.materialIndex;

				if (HitTest_Triangle(triangle, workingRay, hitRecord))
				{
					workingRay.max = hitRecord.t;
				}
			}
			
			if (!hitRecord.didHit) return false;

			hitRecord.materialIndex = mesh.materialIndex;
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