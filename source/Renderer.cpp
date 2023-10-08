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

			Ray hitRay{ camera.origin, rayDirection };

			ColorRGB finalColor{};
			HitRecord closestHit{};

			pScene->GetClosestHit(hitRay, closestHit);
			if (closestHit.didHit)
			{
				for (const Light& light : lights)
				{
					const Vector3 direction{ LightUtils::GetDirectionToLight(light, closestHit.origin) };
					Ray rayToLight{ closestHit.origin + closestHit.normal * 0.001f, direction.Normalized()};
					if (light.type == LightType::Point) rayToLight.max = direction.Magnitude();
					else rayToLight.max = FLT_MAX;

					// Observed area calc + early escape
					const float observedArea{ Vector3::Dot(closestHit.normal, direction) / direction.Magnitude() };
					if (observedArea <= 0.f) continue;

					// Shadows
					if (pScene->DoesHit(rayToLight) && m_ShadowsEnabled) continue;

					// Incident Radiance
					if (m_CurrentLightingMode == LightingMode::Radience)
					{
						finalColor += LightUtils::GetRadiance(light, closestHit.origin);
					}

					// Observed Area
					else if (m_CurrentLightingMode == LightingMode::ObservedArea)
					{
						finalColor += {observedArea, observedArea, observedArea};
					}

					else if (m_CurrentLightingMode == LightingMode::BRDF)
					{
						finalColor += materials[closestHit.materialIndex]->Shade();
					}

					// Combined
					else if (m_CurrentLightingMode == LightingMode::Combined)
					{
						finalColor += LightUtils::GetRadiance(light, closestHit.origin) * materials[closestHit.materialIndex]->Shade() * observedArea;
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
	switch (m_CurrentLightingMode)
	{
	case dae::Renderer::LightingMode::ObservedArea:
		m_CurrentLightingMode = LightingMode::Radience;
		break;
	case dae::Renderer::LightingMode::Radience:
		m_CurrentLightingMode = LightingMode::BRDF;
		break;
	case dae::Renderer::LightingMode::BRDF:
		m_CurrentLightingMode = LightingMode::Combined;
		break;
	case dae::Renderer::LightingMode::Combined:
		m_CurrentLightingMode = LightingMode::ObservedArea;
		break;
	}
}