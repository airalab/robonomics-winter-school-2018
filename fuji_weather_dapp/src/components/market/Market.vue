<template>
  <div>
    <h1 class="text-xs-center">Welcome to the First Robonomics IoT market!</h1>
    <v-container v-if="!robonomicsStatus" fluid fill-height class="px-3">
      <v-layout justify-center align-center>
        <v-flex text-xs-center>
          <h1>Load robonomics</h1>
          <v-progress-circular indeterminate color="primary"></v-progress-circular>
        </v-flex>
      </v-layout>
    </v-container>

    <v-container v-if="robonomicsStatus" grid-list-md class="px-3">
      <v-layout justify-center row wrap>
        <v-flex sm12 md10 lg6>
          <v-card>
            <v-card-title primary-title>
              <v-container grid-list-md>
                <v-layout row wrap>
                  <v-flex md12 class="text-xs-center">
                    <h3 class="headline mb-0 text-xs-center">Digital market channel / Lighthouse:</h3>
                    <b>{{ lighthouseName }}</b>
                  </v-flex>
                </v-layout>
              </v-container>
            </v-card-title>
            <v-card-text>
              <demand-form ref="demandForm"/>
              <v-container v-if="price.value > 0" grid-list-md>
                <v-layout row wrap>
                  <v-flex md12>
                    <div>
                      Cost: {{price.valueStr}} |
                      Balance: {{balance.valueStr}} |
                      Approved: {{approveTrade.valueStr}}
                    </div>
                  </v-flex>
                </v-layout>
              </v-container>

              <div v-if="isOrder" class="text-xs-center">
                <v-btn
                  v-if="approveTrade.value < price.value"
                  :loading="loadingApprove"
                  :disabled="loadingApprove || balance.value < price.value"
                  color="warning"
                  @click.native="sendApproveTrade"
                >Approve</v-btn>
                <v-btn
                  v-if="approveTrade.value >= price.value"
                  :loading="loadingOrder"
                  :disabled="loadingOrder || balance.value < price.value"
                  color="warning"
                  @click.native="order"
                >Order</v-btn>
              </div>

              <div v-else class="text-xs-center">
                <v-btn disabled color="warning">Please, choose offer bellow</v-btn>
              </div>
            </v-card-text>
          </v-card>

          <v-card v-if="liability">
            <v-card-title primary-title>
              <div>
                <h3 class="headline mb-0">Received data</h3>
              </div>
            </v-card-title>
            <v-card-text>
              <Liability :liability="liability"/>
            </v-card-text>
          </v-card>

          <v-card>
            <v-card-title primary-title>
              <v-container grid-list-md>
                <v-layout row wrap>
                  <v-flex md12>
                    <h3 class="headline mb-0 text-xs-center">Offers</h3>
                  </v-flex>
                </v-layout>
              </v-container>
            </v-card-title>
            <v-card-text>
              <v-btn-toggle v-model="toggle_filter" @change="load">
                <v-btn value="h" flat>hour</v-btn>
                <v-btn value="d" flat>day</v-btn>
                <v-btn value="m" flat>month</v-btn>
              </v-btn-toggle>
              <v-divider/>

              <v-progress-linear v-if="offers === null" :indeterminate="true"></v-progress-linear>
              <div v-else>
                <v-list two-line>
                  <v-data-iterator :items="offers" :pagination.sync="pagination" hide-actions>
                    <template slot="item" slot-scope="props">
                      <v-list-tile avatar @click="setForm(props.item.id)">
                        <v-list-tile-avatar>
                          <span v-html="icon(props.item.account)"></span>
                        </v-list-tile-avatar>
                        <v-list-tile-content>
                          <v-list-tile-title v-html="props.item.account"></v-list-tile-title>
                          <v-list-tile-sub-title v-html="props.item.msg.model"></v-list-tile-sub-title>
                        </v-list-tile-content>
                      </v-list-tile>
                      <v-divider/>
                    </template>
                  </v-data-iterator>
                </v-list>
                <v-pagination v-if="pages > 1" v-model="pagination.page" :length="pages"></v-pagination>
              </div>
            </v-card-text>
          </v-card>
        </v-flex>
      </v-layout>
    </v-container>
  </div>
</template>

<script>
import Promise from "bluebird";
import axios from "axios";
import _find from "lodash/find";
import _filter from "lodash/filter";
import _has from "lodash/has";
import { Token } from "robonomics-js";
import getRobonomics from "../../utils/robonomics";
import { formatDecimals } from "../../utils/utils";
import Liability from "./Liability";
import DemandForm from "./DemandForm";
import * as config from "../../config";
import getIpfs, { cat as ipfsCat } from "../../utils/ipfs";
import rosBag from "../../utils/rosBag";
import setSocket from "../../utils/socket";

let robonomics;

export default {
  components: {
    Liability,
    DemandForm
  },
  data() {
    return {
      robonomicsStatus: false,
      token: null,
      price: {
        value: 0,
        valueStr: ""
      },
      loadingApprove: false,
      loadingOrder: false,
      isOrder: false,
      balance: {
        value: 0,
        valueStr: ""
      },
      approveTrade: {
        value: 0,
        valueStr: ""
      },
      lighthouseName: "",
      liability: null,
      offers: [],
      pagination: {
        rowsPerPage: 10
      },
      toggle_filter: "h"
    };
  },
  computed: {
    pages() {
      if (
        this.pagination.rowsPerPage == null ||
        this.pagination.totalItems == null
      )
        return 0;
      return Math.ceil(
        this.pagination.totalItems / this.pagination.rowsPerPage
      );
    }
  },
  created() {
    return getIpfs()
      .then(() => getRobonomics())
      .then(r => {
        robonomics = r;
        return robonomics.ready();
      })
      .then(() => {
        console.log("xrt", robonomics.xrt.address);
        console.log("factory", robonomics.factory.address);
        console.log("lighthouse", robonomics.lighthouse.address);
        this.lighthouseName = robonomics.lighthouse.name;
        robonomics.onDemand(null, msg => {
          console.log("demand", msg);
        });
        robonomics.onResult(msg => {
          console.log("result unverified", msg);
          if (
            robonomics.web3.toChecksumAddress(msg.liability) ===
            robonomics.web3.toChecksumAddress(robonomics.account.address)
          ) {
            this.liability = {
              address: "",
              resultMessage: []
            };
            this.setResult(msg.result, true);
          } else if (
            this.liability !== null &&
            msg.liability === this.liability.address
          ) {
            this.setResult(msg.result, false);
          }
        });
        this.load();
        setSocket(socket => {
          socket.on("add", offer => {
            this.offers.unshift({
              msg: JSON.parse(offer.msg),
              id: offer.id,
              account: offer.account
            });
          });
          socket.on("remove", id => {
            this.offers = _filter(this.offers, offer => {
              return offer.id !== id;
            });
          });
        });
        this.robonomicsStatus = true;
      });
  },
  methods: {
    load() {
      this.offers = null;
      axios.get(config.OFFERS_API + this.toggle_filter).then(r => {
        this.offers = [];
        r.data.result.forEach(offer => {
          this.offers.push({
            msg: JSON.parse(offer.msg),
            id: offer.id,
            account: offer.account
          });
        });
      });
    },
    icon(account) {
      return jdenticon.toSvg(account, 40);
    },
    setForm(id) {
      this.isOrder = false;
      const offer = _find(this.offers, { id });
      this.$refs.demandForm.model = offer.msg.model;
      this.$refs.demandForm.objective = offer.msg.objective;
      this.$refs.demandForm.token = offer.msg.token;
      this.$refs.demandForm.cost = offer.msg.cost;
      this.$refs.demandForm.deadline = offer.msg.deadline;

      this.price.value = Number(offer.msg.cost);
      if (this.price.value > 0) {
        this.token = new Token(robonomics.web3, offer.msg.token);
        Promise.join(
          this.token.call.decimals(),
          this.token.call.symbol(),
          (decimals, symbol) => {
            this.token.decimals = decimals;
            this.token.symbol = symbol;

            this.price.valueStr = `${formatDecimals(
              this.price.value,
              this.token.decimals
            )} ${this.token.symbol}`;
            return this.fetchBalance();
          }
        ).then(() => {
          this.isOrder = true;
        });
      } else {
        this.isOrder = true;
      }
    },
    fetchBalance() {
      return this.token.call
        .balanceOf(robonomics.account.address)
        .then(balanceOf => {
          this.balance.value = balanceOf;
          this.balance.valueStr = `${formatDecimals(
            balanceOf,
            this.token.decimals
          )} ${this.token.symbol}`;
          if (balanceOf > 0) {
            return this.token.call.allowance(
              robonomics.account.address,
              robonomics.factory.address
            );
          }
          return 0;
        })
        .then(allowance => {
          this.approveTrade.value = allowance;
          this.approveTrade.valueStr = `${formatDecimals(
            allowance,
            this.token.decimals
          )} ${this.token.symbol}`;
        });
    },
    sendApproveTrade() {
      this.loadingApprove = true;
      return this.token.send
        .approve(robonomics.factory.address, this.price.value * 100, {
          from: robonomics.account.address
        })
        .then(() => this.fetchBalance())
        .then(() => {
          this.loadingApprove = false;
        })
        .catch(() => {
          this.loadingApprove = false;
        });
    },
    setResult(result, check = true) {
      this.liability = {
        ...this.liability,
        result,
        check
      };
      if (this.liability.resultMessage.length === 0) {
        this.liability.resultMessage.push("");
        ipfsCat(result).then(r => {
          rosBag(
            new Blob([r]),
            bag => {
              try {
                const json = JSON.parse(bag.message.data);
                if (_has(json, "weather")) {
                  this.liability.resultMessage.push({
                    type: 1,
                    json,
                    str: JSON.stringify(json, undefined, 2)
                  });
                } else {
                  this.liability.resultMessage.push({
                    type: 2,
                    str: JSON.stringify(json, undefined, 2)
                  });
                }
              } catch (e) {
                this.liability.resultMessage.push({
                  type: 3,
                  str: bag.message.data
                });
              }
            },
            {}
          );
        });
      }
    },
    newLiability(liability) {
      console.log("liability demand", liability.address);
      return liability
        .getInfo()
        .then(info => {
          this.liability = {
            address: liability.address,
            worker: liability.worker,
            ...info,
            resultMessage: []
          };
          liability.onResult().then(result => {
            console.log("result", result);
            this.setResult(result, true);
          });
          return true;
        })
        .catch(e => {
          console.log(e);
          setTimeout(() => {
            this.newLiability(liability);
          }, 2000);
        });
    },
    order() {
      if (this.$refs.demandForm.$refs.form.validate()) {
        this.liability = null;
        this.loadingOrder = true;
        const demand = {
          model: this.$refs.demandForm.model,
          objective: this.$refs.demandForm.objective,
          token: this.$refs.demandForm.token,
          cost: this.$refs.demandForm.cost,
          lighthouse: robonomics.lighthouse.address,
          validator: "0x0000000000000000000000000000000000000000",
          validatorFee: 0,
          deadline: this.$refs.demandForm.deadline
        };
        robonomics
          .sendDemand(demand)
          .then(liability => this.newLiability(liability))
          .then(() => {
            this.loadingOrder = false;
          })
          .catch(() => {
            this.loadingOrder = false;
          });
      }
    }
  }
};
</script>
