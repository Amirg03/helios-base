// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_move.h"

#include "strategy.h"

#include "bhv_basic_tackle.h"

#include "basic_actions/basic_actions.h"
#include "basic_actions/body_go_to_point.h"
#include "basic_actions/body_intercept.h"
#include "basic_actions/neck_turn_to_ball_or_scan.h"
#include "basic_actions/neck_turn_to_low_conf_teammate.h"

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include "neck_offensive_intercept_neck.h"

#include "player_object.h"
#include "abstract_player_object.h"
#include "localization.h"


#include <rcsc/geom/vector_2d.h>
#include <rcsc/geom/rect_2d.h>
#include <rcsc/math_util.h>
#include <rcsc/geom/polygon_2d.h>

#include <vector>
#include <cstdlib>

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
bool
Bhv_BasicMove::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove" );

    //-----------------------------------------------
    // tackle
    if ( Bhv_BasicTackle( 0.8, 80.0 ).execute( agent ) )
    {
        return true;
    }

    const WorldModel & wm = agent->world();
    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable().selfStep();
    const int mate_min = wm.interceptTable().teammateStep();
    const int opp_min = wm.interceptTable().opponentStep();

    if ( ! wm.kickableTeammate()
         && ( self_min <= 3
              || ( self_min <= mate_min
                   && self_min < opp_min + 3 )
              )
         )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": intercept" );
        Body_Intercept().execute( agent );
        agent->setNeckAction( new Neck_OffensiveInterceptNeck() );

        return true;
    }
    //! ball_nearest_teammate || ( ball_nearest_teammate->distFromBall() > wm.ball().distFromSelf() - 5.0 ) 
    //! wm.kickableTeammate

    /*   std::vector< rcsc::Vector2D > rect;
    rect.emplace_back(  0,  0 );
    rect.emplace_back( 10,  0 );
    rect.emplace_back( 10, 10 );
    rect.emplace_back(  0, 10 );

    const rcsc::Polygon2D r( rect );*/

    
    const PlayerObject * opponent = static_cast< const PlayerObject * >( 0 );

    const int self_min = wm.interceptTable().selfStep();
    const Vector2D self_reach_point = wm.ball().inertiaPoint( self_min );

    const double goal_post_y1 = ServerParam::i().goalHalfWidth();
    const double goal_post_y2 = -ServerParam::i().goalHalfWidth();  
    const double halfPitch = -ServerParam::i().pitchHalfLength();

    std::vector< rcsc::Vector2D > tri;
    tri.emplace_back( -200.0, goal_post_y1);
    tri.emplace_back(    0.0, goal_post_y2 );
    tri.emplace_back( +200.0, -100.0 );

    const rcsc::Polygon2D triangle( tri );


    for ( PlayerObject::Cont::const_iterator o = wm.opponentsFromBall().begin(),
                  end = wm.opponentsFromBall().end();
              o != end;
              ++o )
        {
            if ( (*o)->distFromBall() > 5.0 ) continue;
            if ( (*o)->unum() == Unum_Unknown ) continue;
            if ( (*o)->isGhost()) continue;
            if ( (*o)->goalie()) continue;
            if ( (*o)->isTackling() ) continue;
            if ( (*o)->pos().dist(self_reach_point) > 6) continue;
            //int dash_step = tm->playerTypePtr()->cyclesToReachDistance(dist);
            
            //if ( (*o)->seenPosCount() > 0 ) continue;
            //if ( (*o)->unumCount() > 0 ) continue;
            //if ( (*o)->bodyCount() > 0 ) continue;
            //if ( (*o)->pos().dist2( opp_trap_pos ) > 15.0*15.0 ) continue;
            // if ( (*o)->distFromSelf() > 20.0 ) continue;

            // if ( (*o)->unum() == s_last_sent_opponent_unum
            //      && s_last_opponent_send_time.cycle() >= wm.time().cycle() - 1 )
            // {
            //     continue;
            // }

            opponent = *o;
            break;
        }
    

    const Vector2D target_point = Strategy::i().getPosition( wm.self().unum() );
    const double dash_power = Strategy::get_normal_dash_power( wm );

    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if ( dist_thr < 1.0 ) dist_thr = 1.0;

    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicMove target=(%.1f %.1f) dist_thr=%.2f",
                  target_point.x, target_point.y,
                  dist_thr );

    agent->debugClient().addMessage( "BasicMove%.0f", dash_power );
    agent->debugClient().setTarget( target_point );
    agent->debugClient().addCircle( target_point, dist_thr );

    if ( ! Body_GoToPoint( target_point, dist_thr, dash_power
                           ).execute( agent ) )
    {
        Body_TurnToBall().execute( agent );
    }

    if ( wm.kickableOpponent()
         && wm.ball().distFromSelf() < 18.0 )
    {
        agent->setNeckAction( new Neck_TurnToBall() );
    }
    else
    {
        agent->setNeckAction( new Neck_TurnToBallOrScan( 0 ) );
    }

    return true;
}
