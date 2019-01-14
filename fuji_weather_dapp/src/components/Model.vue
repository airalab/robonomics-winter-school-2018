<template>
  <div>
    <h1 class="text-xs-center">Select weather</h1>
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
        <v-flex xs12 md10 lg6>
          <v-card>
            <v-card-text>
              <v-container grid-list-md class="px-3">
                <v-layout row wrap>
                  <v-flex md12 class="text-xs-center">
                    Lighthouse: <b>{{ lighthouse.name }}</b>
                    <ModelForm ref="modelForm" class="mt-5" />
                    <v-btn
                      :loading="loadingOrder"
                      :disabled="loadingOrder"
                      color="primary"
                      @click.native="order"
                    >
                      Order
                    </v-btn>
                  </v-flex>
                </v-layout>
              </v-container>
            </v-card-text>
          </v-card>
        </v-flex>
      </v-layout>

      <v-layout justify-center row wrap v-if="demand !== null">
        <v-flex xs12 md10 lg6>
          <v-card>
            <v-card-title primary-title>
              <div>
                <h3 class="headline mb-0">Received data</h3>
              </div>
            </v-card-title>
            <v-card-text>
              <v-progress-linear v-if="frees.length === 0" :indeterminate="true"></v-progress-linear>
              <div v-else v-for="(res, resIndex) in frees" :key="resIndex">
                <b>IPFS hash of data: </b>
                <a :href="`https://ipfs.io/ipfs/${res.hash}`" target="_blank">{{ res.hash }}</a>
                <br/>
                <v-progress-linear v-if="res.result.length === 0" :indeterminate="true"></v-progress-linear>
                <div v-else>
                  <div v-for="(item, i) in res.result" :key="i">
                    <div style="border: 1px solid #cccaca;margin: 10px 0;width: 200px;padding: 10px;text-align:center">
                      <img width="64" height="64" :src="`//openweathermap.org/themes/openweathermap/assets/vendor/owm/img/widgets/${item.json.weather[0].icon}.png`">
                      <v-divider />
                      <div style="padding-top: 15px;"><b>{{ parseInt(item.json.main.temp) - 273 }}<span>°C</span></b></div>
                    </div>
                    <code style="width:100%"><pre>{{ item.str }}</pre></code>
                  </div>
                </div>
                <v-divider class="my-3" />
              </div>
            </v-card-text>
          </v-card>
        </v-flex>
      </v-layout>
    </v-container>
  </div>
</template>

<script>
import { Token } from 'robonomics-js'
import _findIndex from 'lodash/findIndex'
import _has from 'lodash/has'
import getRobonomics from '../utils/robonomics'
import ipfsBagCat from '../utils/ipfs'
import * as config from '../config'
import ModelForm from './ModelForm'

let robonomics

export default {
  components: {
    ModelForm
  },
  data () {
    return {
      robonomicsStatus: false,
      token: null,
      loadingOrder: false,
      lighthouse: {
        name: '',
        address: ''
      },
      demand: null,
      frees: [],
      models: {}
    }
  },
  created () {
    getRobonomics()
      .then((r) => {
        robonomics = r
        robonomics.ready().then(() => {
          if (config.TOKEN) {
            this.token = new Token(robonomics.web3, config.TOKEN)
          } else {
            this.token = robonomics.xrt
          }
          this.lighthouse.name = robonomics.lighthouse.name
          this.lighthouse.address = robonomics.lighthouse.address
          this.models[config.MODEL_TRADE] = true
          robonomics.getDemand(config.MODEL_TRADE, (msg) => {
            if (msg.account === robonomics.account) {
              this.demand = msg
              return this.emulatorKfc(msg)
            }
          })
          robonomics.getResult((msg) => {
            console.log('result unverified', msg)
            if (web3.toChecksumAddress(msg.liability) === web3.toChecksumAddress(robonomics.account)) {
              const i = _findIndex(this.frees, (item) => {
                return item.hash === msg.result
              })
              if (i < 0) {
                this.frees.push({
                  hash: msg.result,
                  result: []
                })
                const k = this.frees.length - 1
                ipfsBagCat(msg.result, { topics: ['/data'] }, (bag) => {
                  const json = JSON.parse(bag.message.data)
                  this.frees[k].result.push({
                    json,
                    str: JSON.stringify(json, undefined, 2)
                  })
                })
              }
            }
          })
          this.robonomicsStatus = true
        })
      })
  },
  methods: {
    emulatorKfc (demand) {
      return robonomics.postResult({ liability: robonomics.account, success: true, result: 'QmVAFgUxBitKqtV2sjaYcHkKfcAPVy3GswhaE5n5bcgLkf' })
        .then(() => {
          console.log('result send msg')
        })
    },
    order () {
      if (this.$refs.modelForm.$refs.form.validate()) {
        if (!_has(this.models, this.$refs.modelForm.model)) {
          this.models[this.$refs.modelForm.model] = true
          robonomics.getDemand(this.$refs.modelForm.model, (msg) => {
            if (msg.account === robonomics.account) {
              this.demand = msg
              return this.emulatorKfc(msg)
            }
          })
        }
        this.loadingOrder = true
        web3.eth.getBlock('latest', (e, r) => {
          const demand = {
            objective: config.OBJECTIVE_TRADE,
            token: this.token.address,
            cost: config.PRICE,
            lighthouse: robonomics.lighthouse.address,
            validator: '0x0000000000000000000000000000000000000000',
            validatorFee: 0,
            deadline: r.number + 1000
          }
          robonomics.post('demand', this.$refs.modelForm.model, demand)
            .then(() => {
              this.loadingOrder = false
            })
            .catch((e) => {
              this.loadingOrder = false
            })
        })
      }
    }
  }
}
</script>