#ifndef STAT_H
#define STAT_H

#ifdef _WIN32
#include "render/RenderInterface.h"
#endif

class DecoStat
{
private:
	INT m_width;
	INT m_height;
	INT m_numPrimitive;
	INT m_numObjectRendered;
	INT m_numObjectTotal;
	FLOAT m_renderTime;
	FLOAT m_totalTickTime;
	FLOAT m_interactTime;
	FLOAT m_fps;
	
	static DecoStat * ms_statistics;
	DecoStat() : m_width(0), m_height(0), m_numPrimitive(0), m_numObjectRendered(0), m_numObjectTotal(0), m_renderTime(0), m_totalTickTime(0), m_interactTime(0), m_fps(0)
	{}
	~DecoStat()
	{};
#ifdef _WIN32
	void DrawText(DecoRenderInterface* RI, char * text, UINT posX, UINT posY, INT win_width, INT win_height);
#endif
public:
	static DecoStat* GetSingleton()
	{
		if (!ms_statistics)
		{
			ms_statistics = new DecoStat();
			//			wglUseFontBitmaps(wglGetCurrentDC(),0,256,1000);
		}
		return ms_statistics;
	}
	static void DestroySingleton()
	{
		if (ms_statistics)
			delete ms_statistics;
		ms_statistics = NULL;
	}
	void SetWindowSize(INT width, INT height)
	{
		m_width = width;
		m_height = height;
	}
	void Reset();
#ifdef _WIN32
	void OutputToWindow(DecoRenderInterface* RI);
#endif
	void CumulatePrimitive(INT num)
	{
		m_numPrimitive += num;
	}
	void CumulateObjectTotal(INT num)
	{
		m_numObjectTotal += num;
	}
	void CumulateObjectRendered(INT num)
	{
		m_numObjectRendered += num;
	}
	void SetRenderTime(FLOAT time)
	{
		m_renderTime = time;
	}
	void CumulateInteractTime(FLOAT time)
	{
		m_interactTime += time;
	}
	void SetTickTime(FLOAT time)
	{
		m_totalTickTime = time;
	}
	void SetFPS(FLOAT fps)
	{
		m_fps = fps;
	}
};


#endif
