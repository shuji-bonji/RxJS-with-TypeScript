---
description: Explica operadores relacionados con multicasting de RxJS, incluyendo estrategias prácticas de multicast como usar share, shareReplay, publish y multicast, conversión de cold a hot, entrega eficiente de valores a múltiples suscriptores y prevención de fugas de memoria. Aprende patrones de implementación para optimización de rendimiento con inferencia de tipos TypeScript para compartición de flujos con seguridad de tipos.
---

# Operadores Usados en Multicasting

RxJS proporciona varios operadores dedicados para lograr "multicasting", compartiendo la misma salida de Observable a múltiples suscriptores.

Esta página introduce brevemente los operadores típicos relacionados con multicasting desde la perspectiva **como operadores**,
y organiza su uso y puntos a tener en cuenta.

> ❗ Para explicaciones conceptuales de multicasting, explicaciones estructurales usando Subjects y ejemplos de código concretos,
> consulta [Mecanismo de Multicasting](/es/guide/subjects/multicasting).

## Principales Operadores Relacionados con Multicasting

| Operador | Características | Notas |
|--------------|------|------|
| **[share()](/es/guide/operators/multicasting/share)** | El método multicast más fácil. Internamente equivalente a `publish().refCount()` | Suficiente para muchos casos de uso |
| **[shareReplay()](/es/guide/operators/multicasting/shareReplay)** | Además de multicasting, proporciona valores recientes al resuscribirse | Cuando se requiere reutilización de estado |
| `publish()` + `refCount()` | Configuración multicast con tiempo de ejecución controlable | Configuración clásica y flexible |
| `multicast()` | API de bajo nivel que pasa `Subject` explícitamente | Útil cuando quieres usar un Subject personalizado |

## Comparación de Patrones de Multicasting

| Operador | Características | Caso de Uso |
|------------|------|-------------|
| **[share()](/es/guide/operators/multicasting/share)** | Multicast básico | Uso simultáneo en múltiples componentes |
| **[shareReplay(n)](/es/guide/operators/multicasting/shareReplay)** | Almacena en búfer los últimos n valores | Suscripción tardía/compartición de estado |
| `publish() + refCount()` | Posible control más detallado | Cuando se necesita control avanzado |
| `multicast(() => new Subject())` | Personalización completa | Cuando se necesitan tipos especiales de Subject |

## Precauciones al Usar Multicasting

1. **Comprensión del timing**: Entiende que el valor que recibes depende de cuándo comienza la suscripción
2. **Gestión del ciclo de vida**: Especialmente al usar `refCount`, el flujo se completa cuando el número de suscriptores llega a cero
3. **Manejo de errores**: Si ocurre un error en un Observable multicast, afectará a todos los suscriptores
4. **Gestión de memoria**: Ten en cuenta las fugas de memoria al usar `shareReplay`, etc.
