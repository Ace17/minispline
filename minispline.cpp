// ----------------------------------------------------------------------------
// Spline computation
// ----------------------------------------------------------------------------

#include <assert.h>
#include <vector>

struct Vec3
{
  double x = 0;
  double y = 0;
  double z = 0;

  Vec3 operator + (Vec3 v) const { return { x + v.x, y + v.y, z + v.z }; }
  Vec3 operator * (double s) const { return { x* s, y* s, z* s }; }
};

// return the 't' value for the control point #idx
double cpParam(int idx) { return idx; }

double weight(int idx1, int idx2, double t)
{
  auto t1 = cpParam(idx1);
  auto t2 = cpParam(idx2);
  assert(t1 < t2);

  return (t - t1) / (t2 - t1);
}

template<typename T>
T blend(double alpha, T a, T b)
{
  return a * (1 - alpha) + b * alpha;
}

int mod(int val, int N)
{
  return (val + 10 * N) % N;
}

int findControlPoint(std::vector<Vec3> const& controlPoints, double t)
{
  auto const N = (int)controlPoints.size();
  int r = N - 1;

  while(t < cpParam(r))
    --r;

  return r;
}

Vec3 evaluateSpline(std::vector<Vec3> const& controlPoints, int order, double t)
{
  const int idx = findControlPoint(controlPoints, t);
  assert(idx >= 0);

  auto const N = (int)controlPoints.size();

  std::vector<Vec3> d = controlPoints;

  for(int r = 0; r < order; r++)
  {
    std::vector<Vec3> tmp(N);

    for(int i = idx - order + r; i <= idx; ++i)
    {
      double alpha = weight(i, i + order - r, t);
      tmp[mod(i, N)] = blend(alpha, d[mod(i - 1, N)], d[mod(i, N)]);
    }

    d = tmp;
  }

  return d[idx];
}

// ----------------------------------------------------------------------------
// GUI
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "SDL.h"
#include "SDL_gfxPrimitives.h"

const int WIDTH = 640;
const int HEIGHT = 480;
const double SCALE = 150.0;

// GUI state
int g_selectedFragment = 1;
int g_iGrabbed;
bool g_bGrabbed;

// curve model
std::vector<Vec3> g_controlPoints;
int g_curveOrder = 1;

Vec3 inverseTransform(int x, int y)
{
  auto xx = double(x - WIDTH / 2) / SCALE;
  auto yy = double(y - HEIGHT / 2) / SCALE;
  return Vec3 { xx, yy, 0 };
}

void transform(Vec3 v, int& x, int& y)
{
  x = WIDTH / 2 + int(v.x * SCALE);
  y = HEIGHT / 2 + int(v.y * SCALE);
  // ignore v.z
}

void addControlPoint()
{
  auto const N = g_controlPoints.size();

  auto const radius = sin(N * 1.5) * 0.15 + 1;
  auto const a = -3.0;
  auto const w = M_PI / 10;

  Vec3 v;
  v.x = cos(a + w * N) * radius;
  v.y = sin(a + w * N) * radius + 0.5;
  v.z = 0;

  g_controlPoints.push_back(v);
}

void removeControlPoint()
{
  g_controlPoints.pop_back();
}

void resetCurve()
{
  g_controlPoints.clear();

  for(int i = 0; i < 10; ++i)
    addControlPoint();
}

void drawScreen(SDL_Surface* screen)
{
  SDL_FillRect(screen, NULL, 0);

  auto const N = (int)g_controlPoints.size();

  for(double tt = cpParam(0); tt < cpParam(N); tt += 0.001)
  {
    auto v = evaluateSpline(g_controlPoints, g_curveOrder, tt);
    int sx, sy;
    transform(v, sx, sy);
    pixelColor(screen, sx, sy, 0xFFFFFFFF);
  }

  for(double tt = cpParam(g_selectedFragment); tt < cpParam(g_selectedFragment + 1); tt += 0.001)
  {
    auto v = evaluateSpline(g_controlPoints, g_curveOrder, tt);
    int sx, sy;
    transform(v, sx, sy);
    pixelColor(screen, sx, sy, 0xFF00FFFF);
  }

  for(int i = 0; i < N; i++)
  {
    int couleur = 0x00FFFFFF;

    if(i == g_iGrabbed)
      couleur = 0xFF0000FF;

    int sx, sy;
    transform(g_controlPoints[i], sx, sy);

    lineColor(screen, sx - 2, sy, sx + 2, sy, couleur);
    lineColor(screen, sx, sy - 2, sx, sy + 2, couleur);
  }

  stringColor(screen, 10, 10, "Tab / shift+Tab   : change selected curve fragment", 0xFFFFFFFF);
  stringColor(screen, 10, 20, "PageUp / PageDown : change curve order", 0xFFFFFFFF);
  stringColor(screen, 10, 30, "Inser / Del       : add/del a control point", 0xFFFFFFFF);
  stringColor(screen, 10, 40, "Click / Drag      : move control points", 0xFFFFFFFF);

  char buffer[256];
  sprintf(buffer, "order : %d", g_curveOrder);
  stringColor(screen, 10, 60, buffer, 0xFFFFFFFF);

  SDL_Flip(screen);
}

// event handling

template<typename T>
auto clamp(T val, T min, T max)
{
  if(val < min)
    return min;

  if(val > max)
    return max;

  return val;
}

void onKeyDown(SDLKey key, bool shifted)
{
  const double speed = 0.02;
  switch(key)
  {
  case SDLK_PAGEDOWN:
    g_curveOrder--;
    break;
  case SDLK_PAGEUP:
    g_curveOrder++;
    break;
  case SDLK_INSERT:
    addControlPoint();
    break;
  case SDLK_DELETE:
    removeControlPoint();
    break;
  case SDLK_LEFT:
    g_controlPoints[g_iGrabbed].x -= speed;
    break;
  case SDLK_RIGHT:
    g_controlPoints[g_iGrabbed].x += speed;
    break;
  case SDLK_DOWN:
    g_controlPoints[g_iGrabbed].y += speed;
    break;
  case SDLK_UP:
    g_controlPoints[g_iGrabbed].y -= speed;
    break;
  case SDLK_TAB:
    g_selectedFragment += (shifted ? -1 : +1);
    break;
  default:
    break;
  }

  auto const N = (int)g_controlPoints.size();

  g_iGrabbed = clamp(g_iGrabbed, 0, N - 1);
  g_curveOrder = clamp(g_curveOrder, 0, N - 1);
  g_selectedFragment = clamp(g_selectedFragment, 0, N - 1);
}

void onClick(int x, int y)
{
  double mindist = 100000.0;
  int idmin = 0;

  // find nearest control point
  for(int i = 0; i < (int)g_controlPoints.size(); i++)
  {
    auto v = inverseTransform(x, y);

    double dx = g_controlPoints[i].x - v.x;
    double dy = g_controlPoints[i].y - v.y;
    double dist = dx * dx + dy * dy;

    if(dist < mindist)
    {
      idmin = i;
      mindist = dist;
    }
  }

  g_iGrabbed = idmin;
  g_bGrabbed = true;
}

void onRelease()
{
  g_bGrabbed = false;
}

void onMove(int x, int y)
{
  if(!g_bGrabbed)
    return;

  g_controlPoints[g_iGrabbed] = inverseTransform(x, y);
}

int main()
{
  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_NOPARACHUTE) < 0)
  {
    fprintf(stderr, "Unable to init SDL: %s", SDL_GetError());
    return 1;
  }

  auto screen = SDL_SetVideoMode(WIDTH, HEIGHT, 32, 0);

  if(!screen)
  {
    fprintf(stderr, "Unable to set video mode: %s", SDL_GetError());
    SDL_Quit();
    return 1;
  }

  SDL_ShowCursor(true);
  SDL_WM_SetCaption("B-Spline plotting using De Boor's algorithm", NULL);

  resetCurve();

  bool quit = false;

  while(!quit)
  {
    drawScreen(screen);

    SDL_Event event;
    SDL_WaitEvent(&event);
    switch(event.type)
    {
    case SDL_MOUSEBUTTONDOWN:
      onClick(event.button.x, event.button.y);
      break;
    case SDL_MOUSEBUTTONUP:
      onRelease();
      break;
    case SDL_MOUSEMOTION:
      onMove(event.motion.x, event.motion.y);
      break;
    case SDL_KEYDOWN:
      onKeyDown(event.key.keysym.sym, (event.key.keysym.mod & KMOD_LSHIFT) || (event.key.keysym.mod & KMOD_RSHIFT));

      if(event.key.keysym.sym == SDLK_ESCAPE)
        quit = true;

      break;
    case SDL_QUIT:
      quit = true;
      break;
    }
  }

  SDL_Quit();
  return 0;
}

