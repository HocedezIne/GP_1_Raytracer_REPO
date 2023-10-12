//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const float aspectRatio = m_Width / static_cast<float>(m_Height);
	const float FOV = tanf((camera.fovAngle*TO_RADIANS) / 2);

	for (int px{}; px < m_Width; ++px)
	{
		float xValue{ (2.f * (float(px) + 0.5f) / m_Width - 1.f) * aspectRatio * FOV };

		for (int py{}; py < m_Height; ++py)
		{
			float yValue{ (1.f - 2.f * (float(py) + 0.5f) / m_Height) * FOV };

			Vector3 rayDirection{ xValue, yValue, 1.f};
			rayDirection = camera.CalculateCameraToWorld().TransformVector(rayDirection);
			rayDirection.Normalize();

			Ray viewRay{ camera.origin, rayDirection };

			ColorRGB finalColor{};
			HitRecord closestHit{};

			pScene->GetClosestHit(viewRay, closestHit);
			if (closestHit.didHit)
			{
				for (const Light& light : lights)
				{
					const Vector3 lightDirection{ LightUtils::GetDirectionToLight(light, closestHit.origin) };
					Ray rayToLight{ closestHit.origin + closestHit.normal * 0.001f, lightDirection.Normalized()};
					if (light.type == LightType::Point) rayToLight.max = lightDirection.Magnitude();
					else rayToLight.max = FLT_MAX;

					// Observed area calc + early escape
					const float observedArea{ Vector3::Dot(closestHit.normal, lightDirection) / lightDirection.Magnitude() };
					if (observedArea <= 0.f) continue;

					// Shadows
					if (pScene->DoesHit(rayToLight) && m_ShadowsEnabled) continue;


					switch (m_CurrentLightingMode)
					{
					case dae::Renderer::LightingMode::ObservedArea:
						finalColor += {observedArea, observedArea, observedArea};
						break;
					case dae::Renderer::LightingMode::Radience:
						finalColor += LightUtils::GetRadiance(light, closestHit.origin);
						break;
					case dae::Renderer::LightingMode::BRDF:
						finalColor += materials[closestHit.materialIndex]->Shade(closestHit, rayToLight.direction, -rayDirection);
						break;
					case dae::Renderer::LightingMode::Combined:
						finalColor += LightUtils::GetRadiance(light, closestHit.origin) *
							materials[closestHit.materialIndex]->Shade(closestHit, rayToLight.direction, -rayDirection) *
							observedArea;
						break;
					}
				}
			}

			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void Renderer::CycleLightingMode()
{
	m_CurrentLightingMode = static_cast<LightingMode>((int(m_CurrentLightingMode)+1) % 4);
}