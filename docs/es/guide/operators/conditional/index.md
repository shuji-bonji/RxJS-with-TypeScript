---
description: Los operadores condicionales de RxJS son operadores para tomar decisiones condicionales sobre valores en un flujo, establecer valores por defecto y evaluar condiciones. defaultIfEmpty, every, isEmpty, etc. se pueden usar para implementar escenarios prácticos como procesar flujos vacíos, verificar todos los valores y verificar la existencia con la seguridad de tipos de TypeScript.
---

# Operadores Condicionales

Los operadores condicionales de RxJS se usan para **determinar o evaluar condicionalmente** el valor de un flujo.
Como establecer un valor por defecto para un flujo vacío, o verificar si todos los valores satisfacen una condición,
se pueden utilizar en escenarios prácticos.

Esta página introduce cada operador en tres etapas: "Sintaxis Básica y Operación", "Ejemplos de Uso Típicos" y "Ejemplos de Código Prácticos (con UI)" en la siguiente estructura.

Comprende para qué casos de uso es adecuado cada operador,
y combinarlos te permitirá diseñar procesamiento reactivo que sea más robusto y esté en línea con tus intenciones.

> [!NOTE]
> `iif` y `defer` son **Creation Functions** (funciones de creación de Observable) y no son operadores condicionales. Consulta [Capítulo 3: Creation Functions](/es/guide/creation-functions/) para estos.

## Lista de Operadores

A continuación se muestra una lista de los principales operadores condicionales y sus características.

| Operador | Descripción |
|--------------|------|
| [defaultIfEmpty](./defaultIfEmpty.md) | Valor alternativo cuando no se emite ningún valor |
| [every](./every.md) | Evalúa si todos los valores coinciden con una condición |
| [isEmpty](./isEmpty.md) | Verifica si se emitió algún valor |

> Para **combinaciones prácticas** y **aplicaciones basadas en casos de uso** de operadores, consulta la sección [Casos de Uso Prácticos](./practical-use-cases.md) al final.


## Ten en Cuenta la Integración con Otras Categorías

Los operadores condicionales solo son útiles en combinación con otros operadores de transformación, combinación y utilidad.
Por ejemplo, es común combinarlos con `switchMap` y `catchError` para realizar "cambio de API y procesamiento de recuperación".

Para más casos de uso prácticos, consulta [Casos de Uso Prácticos](./practical-use-cases.md) para explicaciones detalladas.
