<template>
  <div>
    <div v-if="liability.address">
      <b>liability: </b>{{ liability.address }}<br/>
      <b>model: </b>{{ liability.model }}<br/>
      <b>objective: </b>{{ liability.objective }}<br/>
      <b>token: </b>{{ liability.token }}<br/>
      <b>cost: </b>{{ liability.cost }}<br/>
      <b>promisee: </b>{{ liability.promisee }}<br/>
      <b>promisor: </b>{{ liability.promisor }}<br/>
    </div>
    <span v-if="liability.result != ''">
      <b>IPFS hash of data: </b>
      <a :href="`https://ipfs.io/ipfs/${liability.result}`" target="_blank">{{ liability.result }}</a>
      <v-icon v-if="liability.check === true" large color="green darken-2">mdi-check</v-icon>
      <v-icon v-if="liability.check === false" large color="blue-grey darken-2">mdi-alert-circle-outline</v-icon>
      <br/>
      <b>Data: </b>
      <v-progress-linear v-if="liability.resultMessage.length === 0" :indeterminate="true"></v-progress-linear>
      <div v-else>
        <div v-for="(item, i) in liability.resultMessage" :key="i">
          <div v-if="item.type === 1">
            <div style="border: 1px solid #cccaca;margin: 10px 0;width: 200px;padding: 10px;text-align:center">
              <img width="64" height="64" :src="`//openweathermap.org/themes/openweathermap/assets/vendor/owm/img/widgets/${item.json.weather[0].icon}.png`">
              <v-divider />
              <div style="padding-top: 15px;"><b>{{ parseInt(item.json.main.temp) - 273 }}<span>°C</span></b></div>
            </div>
            <code style="width:100%"><pre>{{ item.str }}</pre></code>
          </div>
          <div v-else-if="item.type === 2">
            <code style="width:100%"><pre>{{ item.str }}</pre></code>
          </div>
          <p v-else>{{ item.str }}</p>
        </div>
      </div>
    </span>
    <span v-if="liability.result == ''">
      <b>IPFS hash of data: </b><v-progress-linear :indeterminate="true"></v-progress-linear>
    </span>
  </div>
</template>

<script>
export default {
  name: 'Liability',
  props: ['liability']
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
