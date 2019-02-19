<template>
  <div>
    <h1 class="text-xs-center">Welcome to the First Robonomics IoT market!</h1>
    <v-container v-if="!robonomicsStatus" fluid fill-height class="px-3">
      <v-layout
        justify-center
        align-center
      >
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
              <h3 class="headline mb-0">Create contarct</h3>
            </v-card-title>
            <v-card-text>
              <v-select
                v-model="objective"
                :items="objectives"
                label="Objective"
              ></v-select>
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

              <v-btn
                v-if="approveTrade.value < price.value"
                :loading="loadingApprove"
                :disabled="loadingApprove || balance.value < price.value"
                color="primary"
                @click.native="sendApproveTrade"
              >
                Approve
              </v-btn>
              <v-btn
                v-if="approveTrade.value >= price.value"
                :loading="loadingOrder"
                :disabled="loadingOrder || balance.value < price.value"
                color="primary"
                @click.native="order"
              >
                Order
              </v-btn>
            </v-card-text>
          </v-card>
        </v-flex>
      </v-layout>

      <v-layout justify-center row wrap>
        <v-flex sm12 md10 lg6>
          <v-card>
            <v-card-title primary-title>
              <div>
                <h3 class="headline mb-0">Contact</h3>
              </div>
            </v-card-title>
            <v-card-text>
              <liability-form ref="liabilityForm" />
              <v-btn
                color="primary"
                @click.native="show"
              >
                Show
              </v-btn>
            </v-card-text>
          </v-card>
        </v-flex>
      </v-layout>

      <v-layout justify-center row wrap v-if="liability">
        <v-flex sm12 md10 lg6>
          <Service :liability="liability" />
        </v-flex>
      </v-layout>
    </v-container>
  </div>
</template>

<script>
import { Liability } from 'robonomics-js'
import getRobonomics from '../../utils/robonomics'
import { formatDecimals, watchTx } from '../../utils/utils'
import getIpfs, { cat as ipfsCat } from '../../utils/ipfs'
import rosBag from '../../utils/rosBag'
import Service from './Service'
import LiabilityForm from './LiabilityForm'
import * as config from '../../config'

let robonomics

export default {
  components: {
    Service,
    LiabilityForm
  },
  data () {
    return {
      robonomicsStatus: false,
      objectives: [],
      objective: {},
      token: null,
      price: {
        value: config.PRICE,
        valueStr: `${config.PRICE} ${config.TOKEN_SYMBOL}`
      },
      loadingApprove: false,
      loadingOrder: false,
      balance: {
        value: 0,
        valueStr: `0 ${config.TOKEN_SYMBOL}`
      },
      approveTrade: {
        value: 0,
        valueStr: `0 ${config.TOKEN_SYMBOL}`,
      },
      liability: null
    }
  },
  created () {
    Object.keys(config.RUN.objectives).forEach((r) => {
      this.objectives.push({
        value: config.RUN.objectives[r].objective,
        text: config.RUN.objectives[r].label
      })
    })
    this.objective = config.RUN.objectives['1h'].objective
    return getIpfs()
      .then((ipfs) => {
        // ipfs.pub
        console.log(ipfs);
        return getRobonomics()
      })
      // .then(() => getRobonomics())
      .then((r) => {
        robonomics = r
        return robonomics.ready()
      })
      .then(() => {
        this.token = robonomics.xrt
        console.log('xrt', robonomics.xrt.address)
        console.log('factory', robonomics.factory.address)
        console.log('lighthouse', robonomics.lighthouse.address)
        robonomics.getDemand(config.RUN.model, (msg) => {
          console.log('demand', msg)
          // emulator kfc
          // return this.emulatorKfc(msg)
        })
        robonomics.getOffer(config.RUN.model, (msg) => {
          console.log('offer', msg)
        })
        robonomics.getResult((msg) => {
          console.log('result unverified', msg)
          if (this.liability !== null && msg.liability === this.liability.address) {
            this.setResult(msg.result, false)
          }
        })
        this.fetchData()
          .then(() => {
            this.robonomicsStatus = true
          })
      })
  },
  methods: {
    emulatorKfc (demand) {
      const offer = {
        objective: demand.objective,
        token: demand.token,
        cost: demand.cost,
        validator: demand.validator,
        lighthouse: demand.lighthouse,
        lighthouseFee: 0,
        deadline: demand.deadline
      }
      return robonomics.postOffer(config.RUN.model, offer)
        .then((liability) => {
          console.log('liability offer', liability.address)
          return robonomics.postResult({ liability: liability.address, success: true, result: 'QmSKmcZscjDCkHx4db5mHkhbtmeWfUEMcmTpwhpnwWaLAJ' })
        })
        .then(() => {
          console.log('result send msg')
        })
    },
    fetchData () {
      return this.token.call('balanceOf', [robonomics.account])
        .then((balanceOf) => {
          this.balance.value = balanceOf
          this.balance.valueStr = `${formatDecimals(balanceOf, config.TOKEN_DECIMALS)} ${config.TOKEN_SYMBOL}`
          if (balanceOf > 0) {
            return this.token.call('allowance', [robonomics.account, robonomics.factory.address])
          }
          return false
        })
        .then((allowance) => {
          if (allowance) {
            this.approveTrade.value = allowance
            this.approveTrade.valueStr = `${formatDecimals(allowance, config.TOKEN_DECIMALS)} ${config.TOKEN_SYMBOL}`
          }
        })
    },
    sendApproveTrade () {
      this.loadingApprove = true
      return this.token.send('approve', [robonomics.factory.address, this.price.value * 100], { from: robonomics.account })
        .then((r) => watchTx(r))
        .then(() => {
          this.loadingApprove = false
          return this.fetchData()
        })
        .catch(() => {
          this.loadingApprove = false
        })
    },
    setResult (result, check = true) {
      this.liability = {
        ...this.liability,
        result,
        check
      }
      if (this.liability.resultMessage.length === 0) {
        this.liability.resultMessage.push({
          type: 2,
          str: ''
        })
        ipfsCat(result)
          .then((r) => {
            rosBag(new Blob([r]), (bag) => {
              try {
                const json = JSON.parse(bag.message.data)
                this.liability.resultMessage.push({
                  type: 1,
                  str: JSON.stringify(json, undefined, 2)
                })
              } catch (e) {
                this.liability.resultMessage.push({
                  type: 2,
                  str: bag.message.data
                })
              }
            }, {})
          })
      }
    },
    newLiability (liability) {
      console.log('liability demand', liability.address)
      return liability.getInfo()
        .then((info) => {
          this.liability = {
            address: liability.address,
            worker: liability.worker,
            ...info,
            resultMessage: []
          }
          liability.watchResult((result) => {
            console.log('result', result)
            this.setResult(result, true)
          })
          return true
        })
        .catch((e) => {
          console.log(e)
          setTimeout(() => {
            this.newLiability(liability)
          }, 2000)
        })
    },
    order () {
      this.liability = null
      this.loadingOrder = true
      robonomics.web3.eth.getBlock('latest', (e, r) => {
        const demand = {
          objective: this.objective,
          token: this.token.address,
          cost: this.price.value,
          lighthouse: robonomics.lighthouse.address,
          validator: '0x0000000000000000000000000000000000000000',
          validatorFee: 0,
          deadline: r.number + 100
        }
        robonomics.postDemand(config.RUN.model, demand)
          .then((liability) => {
            this.$refs.liabilityForm.address = liability.address
            setTimeout(() => {
              this.show()
            }, 500);
          })
          .then(() => {
            this.loadingOrder = false
          })
          .catch(() => {
            this.loadingOrder = false
          })
      })
    },
    show () {
      if (this.$refs.liabilityForm.$refs.form.validate()) {
        const liability = new Liability(robonomics.web3, this.$refs.liabilityForm.address, '')
        this.newLiability(liability)
        liability.getInfo()
          .then((r) => {
            console.log(r);
          })
      }
    }
  }
}
</script>
