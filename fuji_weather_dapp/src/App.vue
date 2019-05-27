<template>
  <div id="app">
    <v-app id="inspire">
      <v-content>
        <v-container grid-list-md style="margin: 18px auto 15px;">
          <v-layout justify-center row wrap>
            <v-flex sm12 md10 lg6>
              <v-layout row wrap>
                <v-flex xs12 sm6 class="text-xs-center text-sm-left">
                  <router-link to="/">
                    <img alt src="static/assets/i/logo.svg" style="height: 45px;">
                  </router-link>
                </v-flex>
                <v-flex xs12 sm6 class="text-xs-center text-sm-right">
                  <v-btn to="/" outline>Main</v-btn>
                  <v-btn to="/model" outline>Model</v-btn>
                  <v-btn to="/market" outline>Market</v-btn>
                  <v-btn to="/robot" outline>Robot</v-btn>
                </v-flex>
              </v-layout>
            </v-flex>
          </v-layout>
        </v-container>
        <web3-check>
          <template v-slot:error="props">
            <div v-if="props.error.type === 'network'">
              <DepNetwork/>
            </div>
            <div v-else-if="props.error.type === 'account'">
              <NotAccounts/>
            </div>
            <div v-else>
              <NotWeb3/>
            </div>
          </template>
          <template slot="load">
            <Load/>
          </template>
          <router-view v-if="ready"/>
        </web3-check>
      </v-content>
    </v-app>
  </div>
</template>

<script>
import Web3Check from "vue-web3-check";
import NotWeb3 from "./components/web3/NotWeb3";
import DepNetwork from "./components/web3/DepNetwork";
import NotAccounts from "./components/web3/NotAccounts";
import Load from "./components/web3/Load";
import { initRobonomics } from "./utils/robonomics";
import getIpfs from "./utils/ipfs";

export default {
  name: "App",
  components: {
    NotWeb3,
    DepNetwork,
    NotAccounts,
    Load
  },
  data() {
    return {
      ready: false
    };
  },
  mounted() {
    Web3Check.store.on("load", state => {
      getIpfs().then(ipfs => {
        initRobonomics(ipfs, state.networkId);
        this.ready = true;
      });
    });
  }
};
</script>

<style>
body {
  font-family: "Helvetica Neue", Helvetica, Arial, sans-serif;
  font-size: 14px;
  line-height: 1.42857143;
  color: #333;
  background-color: #f2f2f2;
}
.application.theme--light {
  background: #f2f2f2;
  color: rgba(0, 0, 0, 0.87);
}
.application .theme--light.v-card,
.theme--light .v-card {
  border: 1px solid #707070;
  border-radius: 0;
  box-shadow: none;
}
</style>
