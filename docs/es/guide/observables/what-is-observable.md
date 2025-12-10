---
description: Observable es un concepto central de RxJS y representa un flujo de datos que ocurre a lo largo del tiempo. Se explican las diferencias con Promise, el mecanismo de suscripción y cancelación, la distinción entre cold y hot, y la gestión del ciclo de vida con ejemplos de código prácticos.
---

# ¿Qué es Observable?

[📘 RxJS Oficial: Observable](https://rxjs.dev/api/index/class/Observable)

Observable en RxJS es un componente central que representa "un flujo de datos que ocurre a lo largo del tiempo" y está diseñado basándose en el patrón Observer para manejar el procesamiento asíncrono y dirigido por eventos de manera unificada.

## Rol de Observable

Un Observable actúa como un "productor de datos" que publica múltiples valores a lo largo del tiempo. En contraste, un Observer actúa como un "consumidor" y se suscribe a los valores a través de `subscribe()`.

En el siguiente ejemplo, creamos un **Observable (productor)** llamado `observable$` y un **Observer (consumidor)** se suscribe y recibe valores.

```ts
import { Observable } from 'rxjs';

// Crear Observable (productor)
const observable$ = new Observable<number>(subscriber => {
  // Lógica a ejecutar al suscribirse
  subscriber.next(1);
  subscriber.next(2);
  subscriber.complete();
});

// Observer (consumidor) se suscribe
observable$.subscribe({
  next: value => console.log('Siguiente valor:', value),
  error: err => console.error('Error:', err),
  complete: () => console.log('Completo')
});

// Salida:
// Siguiente valor: 1
// Siguiente valor: 2
// Completo
```

> [!NOTE]
> La función que se pasa como argumento a `new Observable(function)` define la **lógica a ejecutar cuando se suscribe al Observable**. La función en sí no es el productor; el Observable en su conjunto es el productor.

## Tipos de Notificaciones

Observable envía los siguientes tres tipos de notificaciones al Observer:

- `next`: notificación de un valor
- `error`: notificación cuando ocurre un error (no se envían más notificaciones)
- `complete`: notificación de finalización exitosa

Para más información, consulte la sección [Observer en "Ciclo de vida de Observable"](./observable-lifecycle.md#_2-observer-observer).

## Diferencia entre Observable y Promise

| Característica | Observable | Promise |
|---|---|---|
| Múltiples valores | ◯ | ×(Solo uno) |
| Cancelable | ◯(`unsubscribe()`) | × |
| Ejecución perezosa | ◯ | ◯ |
| Síncrono/Asíncrono | Ambos | Solo asíncrono |

La mayor diferencia entre Observable y Promise es "si puede manejar múltiples valores" y "si puede cancelarse a mitad de camino".
Promise es adecuado para procesamiento asíncrono único, mientras que Observable tiene fortalezas en "datos asíncronos que ocurren continuamente" como flujos de eventos.

Observable también es importante en términos de gestión de recursos, como prevenir fugas de memoria y detener comunicaciones innecesarias, ya que las suscripciones se pueden cancelar a mitad del proceso mediante `unsubscribe()`.

Por otro lado, Promise está ampliamente adoptado en la API estándar y se puede escribir de manera intuitiva combinado con `async/await`. Es deseable usar ambos según la aplicación.

## Distinción entre Cold y Hot

Hay dos tipos de Observable en RxJS: "Cold" y "Hot".

- **Cold Observable**: Cada suscriptor tiene su propio flujo de datos, que comienza a ejecutarse al suscribirse. (ej., `of()`, `from()`, `fromEvent()`, `ajax()`)
- **Hot Observable**: Los suscriptores comparten el mismo flujo de datos y los datos continúan fluyendo independientemente de si están suscritos o no. (ej., `Subject`, Observable multicast con `share()`)

Esta distinción tiene un impacto significativo en el compartimiento de datos y la eficiencia de recursos.
Para más información, consulte la sección ["Cold Observable y Hot Observable"](./cold-and-hot-observables.md).

## Observable y Pipeline

El verdadero valor de un Observable se realiza al combinarlo con operadores usando el método `pipe()`.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5);
numbers$.pipe(
  filter(n => n % 2 === 0), // Pasar solo números pares
  map(n => n * 10)          // Multiplicar por 10
).subscribe(value => console.log(value));
// Salida: 20, 40
```

## Ciclo de Vida de Observable

Observable tiene el siguiente ciclo de vida:

1. **Creación** - creación de una instancia de Observable
2. **Suscripción** - comenzar a recibir datos mediante `subscribe()`
3. **Ejecución** - publicar datos (`next`), error (`error`), o finalización (`complete`)
4. **Cancelación** - finalizar suscripción con `unsubscribe()`

Es importante cancelar las suscripciones de Observable que ya no se necesitan para prevenir fugas de recursos.
Para detalles, consulte la sección ["Ciclo de Vida de Observable"](./observable-lifecycle.md).

## Dónde Usar Observable

- Eventos de UI (clics, desplazamiento, operaciones de teclado, etc.)
- Solicitudes HTTP
- Procesamiento basado en tiempo (intervalos y temporizadores)
- WebSocket y comunicación en tiempo real
- Gestión de estado de aplicación

## Resumen

Observable es la base para el manejo flexible y unificado de datos asíncronos. Como concepto central en ReactiveX (RxJS), proporciona una representación concisa del procesamiento asíncrono complejo y flujos de eventos.
