---
description: "shareReplay es un operador de multidifusión de RxJS que, además de multidifundir, almacena en búfer valores pasados y los proporciona a suscriptores retrasados, ideal para situaciones en las que se desea recordar valores pasados y entregarlos a varios suscriptores, como almacenar en caché respuestas de API, compartir información de configuración o gestionar estados. La prevención de fugas de memoria también es posible con las opciones refCount y windowTime, y la inferencia de tipos de TypeScript garantiza un almacenamiento en caché seguro."
---

# shareReplay - compartir caché

El operador `shareReplay()` proporciona multidifusión del mismo modo que `share()`, pero además **recuerda** un número especificado de valores anteriores y los pone a disposición de los suscriptores que se unan más tarde.

Esto permite casos de uso más avanzados, como almacenar en caché las respuestas de la API y compartir el estado.

[Documentación oficial de RxJS - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 Uso básico

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// shareReplay(tamaño del búfer2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente: ${value}`)),
  shareReplay(2) // Más cercano2Almacenamiento en búfer de dos valores
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción - Último2Recibir dos valores');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### Resultados de la ejecución

```
Observer 1 Inicio de la suscripción
Fuente: 0
Observer 1: 0
Fuente: 1
Observer 1: 1
Fuente: 2
Observer 1: 2
Fuente: 3
Observer 1: 3
Observer 2 Inicio de la suscripción - Último2Recibir dos valores
Observer 2: 2  // ← Valores históricos almacenados en buffer
Observer 2: 3  // ← Valores históricos almacenados en buffer
Fuente: 4
Observer 1: 4
Observer 2: 4
```

**PUNTO IMPORTANTE**:.
- Los suscriptores perezosos también reciben inmediatamente los valores pasados almacenados en buffer
- Los valores se almacenan para el tamaño del búfer (dos en este ejemplo)

## 💡 sintaxis de shareReplay()

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### Parámetros.

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// shareReplay(tamaño del búfer2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente: ${value}`)),
  shareReplay(2) // Más cercano2Almacenamiento en búfer de dos valores
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción - Último2Recibir dos valores');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### Objeto de configuración (RxJS 7+)


```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // Si un abonado es0Darse de baja cuando se conviertan
  scheduler?: SchedulerLike;
}
```

## 📊 Diferencia entre share y shareReplay

### comportamiento de share()

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Fuente: ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Resultados**:.

```
Fuente: 0
Observer 1: 0
Fuente: 1
Observer 1: 1
Observer 2 Inicio de la suscripción
Fuente: 2
Observer 1: 2
Observer 2: 2  // ← Los valores pasados (0, 1) no se reciben
```

### comportamiento de shareReplay()

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Fuente: ${value}`)),
  shareReplay(2) // Más cercano2Buffer un valor
);

source$.subscribe(value => console.log(`Observer 1: ${value}`));

setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Resultados**:.

```
Fuente: 0
Observer 1: 0
Fuente: 1
Observer 1: 1
Observer 2 Inicio de la suscripción
Observer 2: 0  // ← Valores históricos almacenados en buffer
Observer 2: 1  // ← Valores históricos almacenados en buffer
Fuente: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Caso práctico.

### 1. Cachear las respuestas de la API.

```typescript
import { Observable } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, shareReplay, tap } from 'rxjs';

interface User {
  id: number;
  name: string;
  username: string;
  email: string;
}

class UserService {
  // Almacenar en caché la información del usuario
  private userCache$ = ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
    tap(() => console.log('APIEjecutar solicitud')),
    shareReplay({ bufferSize: 1, refCount: true }) // Último1Almacenar un valor (refCounty se libera cuando se cancelan todas las suscripciones)
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// Primer componente
userService.getUser().subscribe(user => {
  console.log('Componente1:', user);
});

// 2Segundos después otro componente
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('Componente2:', user); // ← Recuperado de la cachéAPISin solicitud
  });
}, 2000);
```

**Resultados de la ejecución**:.

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// shareReplay(tamaño del búfer2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente: ${value}`)),
  shareReplay(2) // Más cercano2Almacenamiento en búfer de dos valores
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción - Último2Recibir dos valores');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### 2. compartir información de configuración


**Resultados de la ejecución**:.


### 3. caché de tiempo limitado


## ⚠️ Cuidado con las fugas de memoria

shareReplay()` mantiene el valor en un buffer y puede causar fugas de memoria si no se gestiona correctamente.

### Código problemático.


```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// shareReplay(tamaño del búfer2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Fuente: ${value}`)),
  shareReplay(2) // Más cercano2Almacenamiento en búfer de dos valores
);

// Primer abonado
console.log('Observer 1 Inicio de la suscripción');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 3.5Segundos después2Añadir segundo abonado
setTimeout(() => {
  console.log('Observer 2 Inicio de la suscripción - Último2Recibir dos valores');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

### Contramedidas recomendadas


## 🎯 Cómo elegir el tamaño del búfer


## 🔄 Operadores relacionados

- **[share()](/es/guide/operators/multicast/share)** - multicast simple (sin buffer)
- **[publish()](/es/guide/sujetos/multicast)** - control multicast de bajo nivel
- ReplaySubject](/es/guide/subjects/types-of-subjects)** - Subject subyacente de shareReplay

## Resumen.

El operador `shareReplay()` es,
- Almacenar en caché los valores anteriores y proporcionarlos también a los suscriptores perezosos
- Ideal para almacenar en caché las respuestas de la API
- Hay que tener cuidado con las fugas de memoria
- Seguro de usar con `refCount` y `windowTime

Si necesita compartir o almacenar en caché el estado, shareReplay()` es una herramienta muy potente, pero es importante establecer tamaños de búfer y límites de tiempo apropiados.

## 🔗 Secciones relacionadas.

- **[Errores comunes y soluciones](mal uso de /guide/anti-patterns/common-mistakes#4-sharereplay-)** - Uso adecuado de shareReplay y prevención de fugas de memoria.
- **[share()](/es/guide/operators/multicasting/share)** - multicast simple
- ReplaySubject](/es/guide/subjects/types-of-subjects)** - el Subject subyacente de shareReplay
