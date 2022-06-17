//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "RSUControlApp.h"
#include "Constant.h"
#include "Graph.h"
#include <vector>
#include <string>

#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

using namespace veins;

Register_Class(RSUControlApp);

void printVertex(NodeVertex *root) {
    if (root != NULL) {
        printVertex(root->left);
        if (root->j_of_vertex.size() == 0) {
            EV << root->v->getId() << endl;
//            for (auto r : root->v->getFrom()) {
//                EV << r->getId() << " ";
//            }
//            EV << endl;
//            for (auto r : root->v->getTo()) {
//                EV << r->getId() << " ";
//            }
//            EV << endl;
//        if (root->j_of_vertex.size() > 2) {
//            for (auto r : root->v->getInternals()) {
//                if (r->getFrom().length() > 0 && r->getTo().length() > 0)
//                    EV << r->getFrom() << " + " << r->getTo() << " + "
//                              << r->getW() << endl;
//            }
//        }
        }
        printVertex(root->right);
    }
}

void RSUControlApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sendBeacon = new cMessage("send Beacon");
        graph = new Graph();
//        printVertex(graph->getVertex());
    } else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
    }

}

void RSUControlApp::finish() {
    //Duoc goi khi RSU ket thuc cong viec
    DemoBaseApplLayer::finish();
    EV << "Finish here" << endl;
    for (auto i : message) {
        EV << i << endl;
    }
    // statistics recording goes here
}

void RSUControlApp::onBSM(DemoSafetyMessage *bsm) {
    //for my own simulation circle
}

void RSUControlApp::onWSM(BaseFrame1609_4 *wsm) {
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    cPacket *enc = wsm->getEncapsulatedPacket();
    if (TraCIDemo11pMessage *bc = dynamic_cast<TraCIDemo11pMessage*>(enc)) {
//        if(strcmp(Constant::FIRST, bc->getDemoData()) == 0){
//        EV << "my message = " << bc->getDemoData() << " from "
//                  << bc->getSenderAddress() << endl;
        if (sendBeacon != NULL) {
            if (sendBeacon->isScheduled()) {
                cancelEvent(sendBeacon);
            }

            TraCIDemo11pMessage *rsuBeacon = new TraCIDemo11pMessage();

            char *ret = mergeContent(bc->getSenderAddress());
            rsuBeacon->setDemoData(ret);
            rsuBeacon->setSenderAddress(myId);
            BaseFrame1609_4 *WSM = new BaseFrame1609_4();
            WSM->encapsulate(rsuBeacon);
            populateWSM(WSM);
            send(WSM, lowerLayerOut);
        }
        std::stringstream streamData(bc->getDemoData());
        std::string str;
        AGV *cur = NULL;
        for (auto a : vh) {
            if (a->id.compare(std::to_string(bc->getSenderAddress())) == 0)
                cur = a;
        }
        if (cur == NULL) {
            cur = new AGV();
            cur->id = std::to_string(bc->getSenderAddress());
            cur->n = new Count();
            cur->n->stopTime = 0;
            vh.push_back(cur);
        }
        int i = 0;
        while (getline(streamData, str, ' ')) {
            if (i == 0) {
                str.erase(str.find("_"));
                cur->n->laneId = str;
                if (cur->n->beforeLaneId.length() == 0) {
                    cur->n->beforeLaneId = str;
                } else if (cur->n->laneId.compare(cur->n->beforeLaneId) != 0) {
//                    EV << count->laneId << " " << count->k << endl;
                    std::string mes;
                    if (cur->n->beforeLaneId.front() == ':') {
                        mes = cur->n->laneId + " "
                                + std::to_string(cur->n->stopTime * 0.1);
                        NodeVertex *nv = graph->searchVertex(
                                cur->n->beforeLaneId);
                        nv->v->setW(cur->n->stopTime * 0.1);
                        message.push_back(mes);
                    } else if (cur->n->laneId.front() == ':') {
                        std::string full_name = cur->n->beforeLaneId + "-"
                                + str;
                        mes = full_name + " "
                                + std::to_string(cur->n->stopTime * 0.1);
                        NodeVertex *nv = graph->searchVertex(full_name);
                        nv->v->setW(cur->n->stopTime * 0.1);
                        message.push_back(mes);
                    }
                    if (cur->n->beforeLaneId.front() != cur->n->laneId.front())
                        cur->n->stopTime = 0;
                    cur->n->beforeLaneId = cur->n->laneId;
                }
            } else if (i == 2) {
                if (std::stod(str) == 0) {
                    cur->n->stopTime++;
                }
            }
            i++;
        }
    }
}

void RSUControlApp::onWSA(DemoServiceAdvertisment *wsa) {
// Your application has received a service advertisement from another car or RSU
// code for handling the message goes here, see TraciDemo11p.cc for examples
}

void RSUControlApp::handleSelfMsg(cMessage *msg) {
    DemoBaseApplLayer::handleSelfMsg(msg);
}

void RSUControlApp::handlePositionUpdate(cObject *obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
// the vehicle has moved. Code that reacts to new positions goes here.
// member variables such as currentPosition and currentSpeed are updated in the parent class

}

