---
description: El operador takeUntil se utiliza para suscribirse al Observable original hasta que el Observable notificador emita un valor y luego cancelar la suscripción cuando se notifique.
---

# takeUntil

El operador `takeUntil` **mantiene la suscripción al Observable original hasta que el Observable especificado (disparador de notificación) emita su primer valor**. El Observable original se desuscribe en el momento en que el disparador de notificación emite.

## 🔁 Sintaxis Básica

```ts
source$.pipe(
  takeUntil(notifier$)
)
```

- `source$`: Observable original (objetivo de suscripción)
- `notifier$`: Observable que señala detención (la suscripción se detiene cuando este Observable emite su primer valor)

[🌐 Documentación Oficial de RxJS - takeUntil](https://rxjs.dev/api/index/function/takeUntil)

## 🧪 Ejemplo de Uso: Detener Suscripción al Hacer Clic en Botón

```ts
import { interval, fromEvent } from 'rxjs';
import { takeUntil } from 'rxjs';

const stopButton = document.createElement('button');
stopButton.textContent = 'detener';
document.body.appendChild(stopButton)

const stop$ = fromEvent(stopButton, 'click');
const source$ = interval(1000); // Emitir número cada segundo

source$
  .pipe(takeUntil(stop$))
  .subscribe((val) => console.log(`Valor: ${val}`));
```

📌 Cuando se hace clic en `stopButton`, la suscripción a `source$` se detiene en ese momento.

## ✅ Casos de Uso Comunes

- Cuando desea detener solicitudes HTTP o proceso de polling con un botón de cancelación
- Cuando desea cancelar la suscripción de un componente según su ciclo de vida
- Cuando desea terminar procesamiento asíncrono por transición de página o desmontaje

## 🔗 Operadores Relacionados

- `take`: Tomar valores hasta cierto número de veces
- `first`: Obtener solo el primer caso y salir
- `skipUntil`: Ignorar hasta que un Observable particular emita un valor
