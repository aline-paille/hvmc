#include "hvmc_graphics.h"

#include <SDL2/SDL.h>
#include <vector>
#include "hvmc_utils.h"

void GraphicsComponentRender( GraphicsComponent* component, SDL_Renderer* renderer )
{
    SDL_RenderCopyEx( renderer, component->texture, nullptr, component->rect,
                      component->rotation, nullptr, SDL_FLIP_NONE );
}

void GraphicsComponentCleanup( GraphicsComponent* component )
{
    delete component->rect;
    if ( component->texture != nullptr )
    {
        SDL_DestroyTexture( component->texture );
    }
}

bool GraphicsSystem::Init( SDL_Renderer* pRenderer )
{
    renderer = pRenderer;

    wallVertTextureInit = false;
    wallHoriTextureInit = false;
    crateTextureInit = false;
    smileyTextureInit = false;
    
    return true;
}

void GraphicsSystem::Render()
{
    for ( auto& c : components )
    {
        GraphicsComponentRender( c, renderer );
    }
}

GraphicsComponent* GraphicsSystem::AddSphere( vec2 const& pos, f32 radius )
{
    GraphicsComponent* comp = new GraphicsComponent;

    comp->rect = new SDL_Rect;
    comp->rect->x = (i32)pos.x;
    comp->rect->y = (i32)pos.y;
    comp->rect->w = radius * 2.f;
    comp->rect->h = radius * 2.f;
    comp->rotation = 0.f;

    if ( !smileyTextureInit )
    {
        smileyTexture = CreateTextureFromImage( renderer, "assets/smiley.png" );
        smileyTextureInit = true;
    }
    
    comp->texture = smileyTexture;
    
    components.push_back( comp );
    return comp;
}

GraphicsComponent* GraphicsSystem::AddBox( vec2 const& pos, vec2 const& dim )
{
    GraphicsComponent* comp = new GraphicsComponent;

    comp->rect = new SDL_Rect;
    comp->rect->x = (i32)pos.x;
    comp->rect->y = (i32)pos.y;
    comp->rect->w = (i32)dim.x;
    comp->rect->h = (i32)dim.y;
    comp->rotation = 0.f;

    if ( !crateTextureInit )
    {
        crateTexture = CreateTextureFromImage( renderer, "assets/crate.png" );
        crateTextureInit = true;
    }
    
    comp->texture = crateTexture;
    
    components.push_back( comp );
    
    return comp;
}

GraphicsComponent* GraphicsSystem::AddWall( vec2 const& pos, vec2 const& dim )
{
    GraphicsComponent* comp = new GraphicsComponent;

    comp->rect = new SDL_Rect;
    comp->rect->x = (i32)pos.x;
    comp->rect->y = (i32)pos.y;
    comp->rect->w = (i32)dim.x;
    comp->rect->h = (i32)dim.y;
    comp->rotation = 0.f;

    if ( dim.x < dim.y )
    {
        if ( !wallVertTextureInit )
        {
            wallVertTexture = CreateTextureFromImage( renderer, "assets/wall-vert.png" );
            wallVertTextureInit = true;
        }
        
        comp->texture = wallVertTexture;
    }
    else
    {
        if ( !wallHoriTextureInit )
        {
            wallHoriTexture = CreateTextureFromImage( renderer, "assets/wall-hori.png" );
            wallHoriTextureInit = true;
        }
        
        comp->texture = wallHoriTexture;
    }
    
    components.push_back( comp );
    return comp;
}

void GraphicsSystem::Cleanup()
{
    for ( auto& c : components )
    {
        GraphicsComponentCleanup( c );
    }
    
    components.clear();
}
