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

	float aspectRatio = m_Width / static_cast<float>(m_Height);

	for (int px{}; px < m_Width; ++px)
	{
		float xValue{ (2.f * (float(px) + 0.5f) / m_Width - 1.f) * aspectRatio };

		for (int py{}; py < m_Height; ++py)
		{
			float yValue{ 1.f - 2.f * (float(py) + 0.5f) / m_Height };

			Vector3 rayDirection{ xValue, yValue, 1.f};
			rayDirection.Normalize();
			Ray hitRay{ camera.origin, rayDirection };

			// updated to test sphere hit test
			ColorRGB finalColor{};
			HitRecord closestHit{};

			pScene->GetClosestHit(hitRay, closestHit);
			if (closestHit.didHit)
			{
				finalColor = materials[closestHit.materialIndex]->Shade();
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
