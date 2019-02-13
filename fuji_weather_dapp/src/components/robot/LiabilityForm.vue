<template>
  <div>
    <v-form ref="form" v-model="valid">
      <v-text-field
        v-model="address"
        :rules="requireRule"
        label="address"
        required
      ></v-text-field>
    </v-form>
  </div>
</template>

<script>
export default {
  data () {
    return {
      valid: false,
      address: '',
      requireRule: [
        v => !!v || 'Field required'
      ]
    }
  }
}
</script>
