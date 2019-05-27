// The Vue build version to load with the `import` command
// (runtime-only or standalone) has been set in webpack.base.conf with an alias.
import '@babel/polyfill';
import Vue from 'vue';
import Vuetify from 'vuetify';
import App from './App';
import router from './router';
import Web3Check, { ACTIONS } from 'vue-web3-check';
import * as config from './config';

import 'vuetify/dist/vuetify.min.css';

Vue.config.productionTip = false;
Vue.use(Vuetify);

Web3Check.store.on('update', data => {
  if (
    (data.state.old.account !== null &&
      data.action.type === ACTIONS.UPD_ACCOUNT) ||
    (data.state.old.networkId !== null &&
      data.action.type === ACTIONS.UPD_NETWORK_ID)
  ) {
    window.location.reload(false);
  }
});
const networks = Object.keys(config.ROBONOMICS).map(item => {
  return Number(item);
});
Vue.use(Web3Check, { Web3, networks: networks, requireAccount: true });

new Vue({
  router,
  render: h => h(App)
}).$mount('#app');
