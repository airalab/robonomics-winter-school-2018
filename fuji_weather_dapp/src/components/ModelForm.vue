<template>
  <div>
    <v-form ref="form" v-model="valid">
      <v-text-field
        v-model="model"
        :rules="modelRules"
        label="Model"
        required
      ></v-text-field>
    </v-form>
  </div>
</template>

<script>
import * as config from '../config'

export default {
  data () {
    return {
      valid: false,
      model: config.MODEL_TRADE,
      modelRules: [
        v => !!v || 'Field is required',
        v => (v.length === 46 && /^Qm/.test(v)) || 'Model must be valid'
      ]
    }
  }
}
</script>
