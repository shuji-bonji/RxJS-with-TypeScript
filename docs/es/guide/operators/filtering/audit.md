---
description: El operador audit es un operador de filtrado de RxJS que emite solo el último valor dentro de un período controlado por un Observable personalizado. Es ideal para control de tiempo dinámico.
titleTemplate: ':title'
---

# audit - Ultimo valor en trigger

El operador `audit` espera a que un Observable personalizado emita un valor y emite el **último valor** de la fuente durante ese período.
Mientras que `auditTime` controla con un tiempo fijo, `audit` puede **controlar el período dinámicamente con un Observable**.

## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento de clic
const clicks$ = fromEvent(document, 'click');

// Separar período cada segundo
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('El clic fue registrado');
});
```

- Cuando ocurre un clic, comienza un período de 1 segundo.
- Solo el último clic durante ese 1 segundo se emite.
- El siguiente período comienza después de 1 segundo.

[🌐 Documentación Oficial de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] Atención en código de producción
> El ejemplo anterior omite la cancelación de suscripción de `fromEvent` para simplificar la explicación. En código real, gestione explícitamente el ciclo de vida con `takeUntil(destroy$)`, `take(N)`, o `Subscription.unsubscribe()`. Detalles: [Superar dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Patrones de Uso Típicos

- **Muestreo a intervalos dinámicos**: Ajustar período según la carga
- **Control de tiempo personalizado**: Control de período basado en otros Observables
- **Limitación adaptativa de eventos**: Reducción según las circunstancias

## 🔍 Diferencia con auditTime

| Operador | Control de Período | Caso de Uso |
|:---|:---|:---|
| `auditTime` | Tiempo fijo (milisegundos) | Control simple basado en tiempo |
| `audit` | **Observable personalizado** | **Control de período dinámico** |

## 📚 Operadores Relacionados

- **[auditTime](/es/guide/operators/filtering/auditTime)** - Controlar con tiempo fijo (versión simplificada de `audit`)
- **[throttle](/es/guide/operators/filtering/throttleTime)** - Emitir primer valor al inicio del período
- **[debounce](/es/guide/operators/filtering/debounceTime)** - Emitir valor después del silencio
- **[sample](/es/guide/operators/filtering/sampleTime)** - Muestrear en el momento de otro Observable

## Resumen

El operador `audit` emite el último valor dentro de un período controlado dinámicamente por un Observable personalizado.

- ✅ Control de período dinámico posible
- ✅ Muestreo adaptativo según la carga
- ✅ Control basado en otros flujos
- ⚠️ Debe generar nuevo Observable cada vez
- ⚠️ Tenga en cuenta la memoria con emisiones frecuentes
