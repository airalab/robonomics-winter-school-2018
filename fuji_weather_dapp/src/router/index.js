import Vue from 'vue'
import Router from 'vue-router'
import All from '@/components/All'
import Model from '@/components/Model'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'All',
      component: All
    },
    {
      path: '/model',
      name: 'Model',
      component: Model
    }
  ]
})
