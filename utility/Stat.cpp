#include "stdafx.h"
#include "Stat.h"
#include <GL/gl.h>
#include <GL/glu.h>

DecoStat* DecoStat::ms_statistics = NULL;


void DecoStat::Reset()
{
	m_numPrimitive = 0;
	m_numObjectRendered = 0;
	m_numObjectTotal = 0;
	m_renderTime = 0;
	m_totalTickTime = 0;
	m_interactTime = 0;
	m_fps = 0;
}

#ifdef _WIN32
void DecoStat::OutputToWindow(DecoRenderInterface* RI)
{
	const INT CHAR_HEIGHT = 15;
	INT currentX = CHAR_HEIGHT;
	INT currentY = m_height - CHAR_HEIGHT;

	RI->SetColor(0xff0000ff);

#ifdef _DEBUG	
	char text[100];

	sprintf(text, "fps: %.4f\0", m_fps);
	DrawText(RI, text, currentX, currentY, m_width, m_height);
	currentY -= CHAR_HEIGHT;

	sprintf(text, "RenderTime: %.2f, InteractTime: %.2f  TotalTime: %.2f\0", m_renderTime, m_interactTime, m_totalTickTime);
	DrawText(RI, text, currentX, currentY, m_width, m_height);
	currentY -= CHAR_HEIGHT;

	sprintf(text, "ObjectRendered: %d, PrimitiveRendered: %d, ObjectTotal:%d\0", m_numObjectRendered, m_numPrimitive, m_numObjectTotal);
	DrawText(RI, text, currentX, currentY, m_width, m_height);
	currentY -= CHAR_HEIGHT;
#endif

}

void DecoStat::DrawText(DecoRenderInterface* RI, char * text, UINT posX, UINT posY, INT win_width, INT win_height)
{
	RI->PushState();

	matrix44 identity = IdentityMatrix44();
	RI->SetTransform(TT_CameraToScreen, identity);
	RI->SetTransform(TT_WorldToCamera, identity);
	RI->SetTransform(TT_LocalToWorld, identity);
	gluOrtho2D(0, win_width,	0, win_height);

	RI->ResetLight();
	RI->EnableDepthTest(FALSE);
	RI->EnableTexture(FALSE);

	glListBase(1000);
	glRasterPos3d(posX,posY,0);
	glCallLists((GLsizei)strlen(text),GL_UNSIGNED_BYTE,text); 

	RI->PopState();
}
#endif
