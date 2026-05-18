---
description: "El operador skipUntil omite todos los valores del Observable original hasta que otro Observable emite un valor, tras lo cual el valor se emite de forma normal. Esto es útil para inicios retardados basados en el tiempo o después de que se haya producido un evento específico."
---

# skipUntil - saltar al encendido

El operador `skipUntil` **salta todos los valores del Observable** original hasta que el primer valor es emitido por el Observable especificado (trigger de notificación). Después del momento en que se emite el disparador de notificación, los valores se emiten como de costumbre.

## 🔰 Sintaxis básica y uso

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5Emitir valor cada segundo
const notifier$ = timer(2000); // 2Emitir valor cada segundo

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Salida: 4, 5, 6, 7, 8, ...
// (primer2segundo valor 0, 1, 2, 3 se omiten)
```

**Flujo de operación**:.
1. `source$` emite 0, 1, 2, 3 → omitir todo
2. 2 segundos después `notifier$` emite un valor
3. los siguientes valores de `source$` (4, 5, 6, ...) se emiten como de costumbre.

[Documentación oficial de RxJS - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)

## 🆚 Contraste con takeUntil

`skipUntil` y `takeUntil` tienen comportamientos opuestos.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // 0.5Emitir valor cada segundo
const notifier$ = timer(2000); // 2Emitir valor cada segundo

// takeUntil: Recuperar valor hasta que se notifique
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Salida: 0, 1, 2, 3(se detiene después de2(se detiene después de 1,5 segundos)

// skipUntil: Omitir valores hasta que se notifique
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Salida: 4, 5, 6, 7, ...(se detiene después de2(Se inicia después de 1,5 segundos)
```

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5Emitir valor cada segundo
const notifier$ = timer(2000); // 2Emitir valor cada segundo

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Salida: 4, 5, 6, 7, 8, ...
// (primer2segundo valor 0, 1, 2, 3 se omiten)
```

```0___

## 💡 Patrón típico de utilización

1. **Inicio del procesamiento de datos tras la autenticación del usuario**.

```

```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // Omitir datos hasta que se complete la autenticación
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`Procesamiento de datos: ${data}`);
   });

   // 3Autenticación completada tras 2 segundos
   setTimeout(() => {
     console.log('Autenticación completada！');
     authenticated$.next();
   }, 3000);
   // 3(Comienza después de 2 segundos) "Tratamiento de datos: 3Tratamiento de datos: 4', "Tratamiento de datos...y salida
   ```

2. **El procesamiento de eventos comienza tras la finalización de la carga inicial**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'Clics.';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // Ignora los clics hasta que la aplicación esté lista
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('Clic procesado');
   });

   // 2Aplicación lista en segundos
   setTimeout(() => {
     console.log('La aplicación está lista');
     appReady$.next(true);
   }, 2000);
   ```

3. **Retraso basado en temporizador iniciado**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { skipUntil, scan } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Cuenta';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');
   const startTime$ = timer(3000); // 3Segundos transcurridos

   // 3Los clics no se cuentan hasta que transcurren segundos
   clicks$.pipe(
     skipUntil(startTime$),
     scan(count => count + 1, 0)
   ).subscribe(count => {
     console.log(`Cuenta: ${count}`);
   });

   console.log('3El recuento comienza pasados los segundos...');
   ```

## 🧠 Ejemplo práctico de código (cuenta atrás del juego)

Este es un ejemplo de cómo ignorar los clics durante la cuenta atrás antes de que empiece el juego y activar los clics después de que termine la cuenta atrás.

```

ts.
import { fromEvent, timer, interval } from 'rxjs';
import { skipUntil, take, scan } from 'rxjs';

// Creación de elementos UI
const container = document.createElement('div');.
document.body.appendChild(contenedor);

const countdown = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'Cuenta atrás en curso...' ;
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = '¡Haz clic! ;
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'puntuación: 0';
container.appendChild(scoreDisplay);

// Cuenta atrás (3 segundos)
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `${3 - n} segundos para empezar... `;
  },.
  complete: () => {
    countdown.textContent = `¡Comienza el juego! ;
    button.disabled = false;
  }
});

// Notificación de inicio de juego
const gameStart$ = timer(3000);.

// Evento click (salta al inicio del juego)
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),.
  scan(puntuación => puntuación + 10, 0)
).subscribe(puntuación => {
  scoreDisplay.textContent = `puntuación: ${puntuación}`;
});

```

En este código, la cuenta atrás3segundos, los clics se ignoran durante la cuenta atrás, y sólo los clics después de que la cuenta atrás termine se reflejan en la puntuación.

## 🎯 skip La diferencia entre skipUntil Diferencia entre

```

ts.
import { interval, timer } from 'rxjs';
import { skip, skipUntil } from 'rxjs';

const fuente$ = interval(500);.

// skip: salta el primer N por número
fuente$.pipe(
  skip(3)
).subscribe(console.log);
// salida: 3, 4, 5, 6, ...

// skipUntil: saltar hasta que se dispare otro Observable
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);.
// Salida: 3, 4, 5, 6, ... (mismo resultado, pero diferente método de control)

```

| Operador | Saltar condiciones | Caso de uso |
|---|---|---|
| `skip(n)` | PrimeronOmitir un número de piezas | Omitir un número fijo |
| `skipWhile(predicate)` | Omitir mientras se cumplen las condiciones | Salto basado en condiciones |
| `skipUntil(notifier$)` | Saltar hasta otroObservableSaltar hasta un | Evento/Omisión basada en el tiempo |

## 📋 Uso seguro

TypeScript Este es un ejemplo de una implementación de tipo seguro que hace uso de los genéricos en

```

ts.
import { Observable, Subject, fromEvent } from 'rxjs';
import { skipUntil, map } from 'rxjs';

interfaz GameState {
  status: 'esperando' | 'listo' | 'jugando' | 'terminado';
}

interfaz ClickEvent {
  timestamp: número; }
  x: número
  y: número;
}

clase Juego {
  private gameReady$ = new Subject();
  private estado: GameState = { estado: 'esperando' };.

  startGame(element: HTMLElement): Observable {
    const clicks$ = fromEvent\<MouseEvent>(element, 'click').pipe(
      map(evento => ({
        timestamp: Date.now(),.
        x: evento.clienteX, evento.
        y: event.clienteY
      } as ClickEvent))),.
      skipUntil(this.gameReady$)
    );

    // Notificación de preparación
    setTimeout(() => {
      this.state = { status: 'ready' };
      this.gameReady$.next();
      console.log('¡Juego listo!') ;
    }, 2000);

    return clicks$;
  }
}

// Ejemplo de uso
const juego = nuevo Juego();
const canvas = document.createElement('div');
canvas.style.width = '300px';
canvas.style.height = '200px';
canvas.style.border = '1px negro sólido';
canvas.textContent = 'Haz clic aquí';
document.body.appendChild(canvas);

game.startGame(canvas).subscribe(click => {
  console.log(`Posición del click: (${click.x}, ${click.y})`);
});

```

## 🔄 skipUntil La diferencia entre takeUntil Combinación de

Combine ambos si sólo desea obtener valores para un periodo de tiempo específico.

```

ts.
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const fuente$ = intervalo(500);
const start$ = timer(2000); // comienza después de 2 segundos
const stop$ = timer(5000); // para después de 5 segundos

fuente$.pipe(
  skipUntil(start$), // saltar hasta después de 2 segundos
  takeUntil(stop$); // parar después de 5 segundos
).subscribe({
  next: console.log,.
  complete: () => console.log('complete')
});
// Salida: 4, 5, 6, 7, 8, 9, completo.
// (sólo se recuperan valores entre 2 y 5 segundos)

```

**Líneas temporales**:
```

0s 1s 2s 3s 4s 5s

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // 0.5Emitir valor cada segundo
const notifier$ = timer(2000); // 2Emitir valor cada segundo

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Salida: 4, 5, 6, 7, 8, ...
// (primer2segundo valor 0, 1, 2, 3 se omiten)
```

```1___

0 1 2 3 4 5 6 7 8 9 10
      ↑ arriba arriba arriba arriba arriba
   SKIP inicio TAKE fin
   (desde 4) (hasta 9)

```

## ⚠️ Un error común

> [!IMPORTANT]
> `skipUntil` son las notificaciones Observable del**Sólo el primer disparo**es válida.2El segundo y posteriores disparos son ignorados.

### Falso: NotificaciónObservablese dispara más de una vez.

```

ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const fuente$ = interval(500);
const notificador$ = new Subject();

source$.pipe(
  skipUntil(notificador$)
).subscribe(console.log);

// ❌ Mal ejemplo: llamar a next varias veces, pero sólo la primera tiene efecto.
setTimeout(() => notifier$.next(), 1000);
setTimeout(() => notificador$.next(), 2000); // esto no tiene sentido

```

### Correcto.: Entiéndase que sólo es válido el primer disparo

```

ts.
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const fuente$ = interval(500);
const notificador$ = new Subject();

source$.pipe(
  skipUntil(notificador$)
).subscribe(console.log);

// ✅ Buen ejemplo: llamar a next sólo una vez
setTimeout(() => {
  console.log('Fin del salto');
  notifier$.next();
  notifier$.complete(); // buena práctica para completar.
}, 1000);
```

## 🎓 Resumen

### Cuándo debe usarse skipUntil.
- ✅ Si desea iniciar el procesamiento después de que se produzca un evento específico.
- ✅ Si desea habilitar las operaciones de usuario después de la inicialización se ha completado
- ✅ Si necesita un inicio diferido en función del tiempo
- ✅ Si desea iniciar el procesamiento de datos una vez finalizada la autenticación

### En combinación con takeUntil.
- ✅ Si desea obtener valores sólo durante un período de tiempo específico (skipUntil + takeUntil).

### Notas.
- ⚠️ Sólo es válido el primer disparo del Observable.
- ⚠️ Si el Observable no se dispara, todos los valores seguirán saltando
- ⚠️ La suscripción se mantiene hasta que se completa el flujo original

## 🚀 Próximos pasos.

- **[skip](. /skip)** - aprende a saltar los N primeros valores.
- **[take](. /take)** - aprende a obtener los N primeros valores.
- **[takeUntil](. /utility/takeUntil)** - aprende a tomar valores hasta que se dispara otro Observable.
- filter](. /filter)** - aprende a filtrar en base a condiciones
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - aprende casos de uso reales
