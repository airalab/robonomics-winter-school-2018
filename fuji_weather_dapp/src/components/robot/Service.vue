<template>
  <v-card>
    <v-card-title primary-title>
      <div>
        <h3 class="headline mb-0">Liability <a :href="`https://etherscan.io/address/${liability.address}`" target="_blank">{{ liability.address }}</a></h3>
      </div>
    </v-card-title>
    <v-card-text>
      <div v-if="liability.isFinalized">
        <span v-if="liability.result != ''">
          <b>Results: </b>
          <a :href="`https://ipfs.io/ipfs/${liability.result}`" target="_blank">{{ liability.result }}</a>
          <v-icon v-if="liability.check === true" large color="green darken-2">mdi-check</v-icon>
          <v-icon v-if="liability.check === false" large color="blue-grey darken-2">mdi-alert-circle-outline</v-icon>
          <br/>
          <span v-for="(res, resIndex) in liability.resultMessage" :key="resIndex">
            {{ res }}<br />
          </span>
        </span>
        <span v-if="liability.result == ''">
          <b>Results: </b><v-progress-linear :indeterminate="true"></v-progress-linear>
        </span>
      </div>
      <div v-else>
        <span v-for="(action, i) in actions" :key="i">
          <v-btn
            :loading="action.loading"
            :disabled="action.loading"
            color="warning"
            @click.native="runAction(i)"
          >
            {{action.name}}
          </v-btn>
        </span>
        <v-divider />
        <div style="margin: 10px 0">
          <h2>Logs</h2>
          <div style="border: 1px solid #6b6060; height: 200px; overflow-y: scroll;">
            <ol>
              <li v-for="(log, i) in logs" :key="i" style="padding: 5px;" :style="{ backgroundColor: (log.type === 1) ? '#f8f8f8' : '#eeeeee' }">
                {{log.msg}}
              </li>
            </ol>
          </div>
        </div>
      </div>
    </v-card-text>
  </v-card>
</template>

<script>
import _find from 'lodash/find'
import _has from 'lodash/has'
import getRobonomics from '../../utils/robonomics'
import getIpfs, { cat as ipfsCat } from '../../utils/ipfs'
import rosBag from '../../utils/rosBag'
import * as config from '../../config'

let robonomics

export default {
  props: ['liability'],
  data () {
    return {
      actions: [],
      logs: []
    }
  },
  created () {
    Object.keys(config.ACTION.objectives).forEach((r) => {
      this.actions.push({
        loading: false,
        name: config.ACTION.objectives[r].label,
        model: config.ACTION.model,
        objective: config.ACTION.objectives[r].objective,
        result: 'QmSKmcZscjDCkHx4db5mHkhbtmeWfUEMcmTpwhpnwWaLAJ',
      })
    })
    // this.actions.push({
    //   loading: false,
    //   name: 'action 1',
    //   model: 'QmXs2aHHeCrQEKXxgBHf2Ty24aJayQDfv1BP1tFRWyUze1',
    //   objective: 'QmXs2aHHeCrQEKXxgBHf2Ty24aJayQDfv1BP1tFRWyUze1',
    //   result: 'QmSKmcZscjDCkHx4db5mHkhbtmeWfUEMcmTpwhpnwWaLAJ',
    // })
    // this.actions.push({
    //   loading: false,
    //   name: 'action 2',
    //   model: 'QmXs2aHHeCrQEKXxgBHf2Ty24aJayQDfv1BP1tFRWyUze2',
    //   objective: 'QmXs2aHHeCrQEKXxgBHf2Ty24aJayQDfv1BP1tFRWyUze2',
    //   result: 'QmSKmcZscjDCkHx4db5mHkhbtmeWfUEMcmTpwhpnwWaLAJ',
    // })
    return getIpfs()
      .then(() => getRobonomics())
      .then((r) => {
        robonomics = r
        return robonomics.ready()
      })
      .then(() => {
        robonomics.getDemand(null, (msg) => {
          console.log('demand', msg)
          const action = _find(this.actions, { model: msg.model })
          if (action) {
            // emulator kfc
            // return this.emulatorKfc(action)
          }
        })
        robonomics.getResult((msg) => {
          console.log('result', msg)
          if (robonomics.web3.toChecksumAddress(msg.liability) === robonomics.web3.toChecksumAddress(robonomics.account)) {
            this.setResult(msg.result)
          }
        })
      })
  },
  methods: {
    emulatorKfc (action) {
      robonomics.postResult({ liability: robonomics.account, success: true, result: action.result })
        .then(() => {
          console.log('result send msg')
        })
    },
    runAction (id) {
      this.actions[id].loading = true
      robonomics.web3.eth.getBlock('latest', (e, r) => {
        const demand = {
          objective: this.actions[id].objective,
          token: robonomics.xrt.address,
          cost: 0,
          lighthouse: robonomics.lighthouse.address,
          validator: '0x0000000000000000000000000000000000000000',
          validatorFee: 0,
          deadline: r.number + 100
        }
        robonomics.post('demand', this.actions[id].model, demand)
          .then(() => {
            this.actions[id].loading = false
            this.logs.push({
              type: 1,
              msg: `Run action "${this.actions[id].name}"`
            })
          })
          .catch(() => {
            this.actions[id].loading = false
          })
      })
    },
    setResult (result) {
      let actionId = 0
      let actionStatus = false
      ipfsCat(result)
        .then((r) => {
          return rosBag(new Blob([r]), (bag) => {
            if (bag.topic === '/agent/objective/id_serial') {
              if (bag.message.data === 'test') {
                actionId = 1
              }
            } else if (bag.topic === '/agent/objective/drone_type') {
              if (bag.message.data === 'ddd') {
                actionStatus = true
              }
            }
          }, {})
        })
        .then(() => {
          if (_has(this.actions, actionId)) {
            if (actionStatus) {
              this.logs.push({
                type: 2,
                msg: `Done action "${this.actions[actionId].name}"`
              })
            } else {
              this.logs.push({
                type: 2,
                msg: `Not done action "${this.actions[actionId].name}"`
              })
            }
          }
        })
    }
  }
}
</script>

<style lang="css">
.fill-row {
  flex: 0 0 100%;
  margin-top: 20px;
  margin-bottom: 20px;
}
#div-with-loading {
  width: 200px;
  height: 200px;
  margin: auto;
  display: flex;
  justify-content: center;
  align-items: center;
  border: 1px solid #444;
  border-radius: 5px;
}
</style>
