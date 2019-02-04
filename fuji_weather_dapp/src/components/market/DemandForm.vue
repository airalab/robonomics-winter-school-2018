<template>
  <div>
    <v-form ref="form" v-model="valid">
      <v-text-field
        v-model="model"
        :rules="requireRule"
        label="model"
        required
      ></v-text-field>
      <v-text-field
        v-model="objective"
        :rules="requireRule"
        label="objective"
        required
      ></v-text-field>
      <v-text-field
        v-model="token"
        :rules="requireRule"
        label="token"
        required
      ></v-text-field>
      <v-text-field
        v-model="cost"
        label="cost"
        required
      ></v-text-field>
      <v-text-field
        v-model="deadline"
        :rules="requireRule"
        label="deadline"
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
      model: '',
      objective: '',
      token: '',
      cost: '',
      deadline: '',
      requireRule: [
        v => !!v || 'Field required'
      ]
    }
  }
}
</script>
