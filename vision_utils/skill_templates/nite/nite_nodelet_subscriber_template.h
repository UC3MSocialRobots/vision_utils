/*!
  \file        nite_nodelet_receiver_template.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/7

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.

\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription

\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication

 */

#ifndef NITE_NODELET_RECEIVER_TEMPLATE_H
#define NITE_NODELET_RECEIVER_TEMPLATE_H

#include <nodelet/nodelet.h>
#include "vision_utils/skill_templates/nite/nite_subscriber_template.h"

class NiteNodeletSubscriberTemplate : public nodelet::Nodelet, public NiteSubscriberTemplate {
public:

  virtual void onInit() {
    NODELET_INFO("onInit()");
    NiteSubscriberTemplate::init();
  }

}; // end class NiteNodeletSubscriberTemplate


#endif // NITE_NODELET_RECEIVER_TEMPLATE_H
