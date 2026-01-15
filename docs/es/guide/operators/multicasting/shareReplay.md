---
description: shareReplay es un operador multicast de RxJS que almacena en búfer valores pasados además de multicasting y los proporciona a suscriptores retrasados. Es ideal para situaciones donde deseas recordar valores pasados y distribuirlos a múltiples suscriptores, como almacenamiento en caché de respuestas API, compartir información de configuración y gestión de estado. Es posible prevenir fugas de memoria con las opciones refCount y windowTime, y la inferencia de tipos TypeScript permite el procesamiento de caché con seguridad de tipos.
---

# shareReplay - Cache y compartir

El operador `shareReplay()` logra multicasting como `share()`, pero también **recuerda un número especificado de valores pasados** y los proporciona a suscriptores que se unen más tarde.

Esto permite casos de uso más avanzados como almacenamiento en caché de respuestas API y compartición de estado.

[📘 RxJS Official Documentation - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 Uso Básico

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Usando shareReplay (tamaño de búfer 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source: ${value}`)),
  shareReplay(2) // Almacenar en búfer los últimos 2 valores
);

// Primer suscriptor
console.log('Suscripción del Observador 1 iniciada');
source$.subscribe(value => console.log(`Observador 1: ${value}`));

// Añadir segundo suscriptor después de 3.5 segundos
setTimeout(() => {
  console.log('Suscripción del Observador 2 iniciada - recibe los últimos 2 valores');
  source$.subscribe(value => console.log(`Observador 2: ${value}`));
}, 3500);
```

### Resultado de Ejecución

```
Suscripción del Observador 1 iniciada
Source: 0
Observador 1: 0
Source: 1
Observador 1: 1
Source: 2
Observador 1: 2
Source: 3
Observador 1: 3
Suscripción del Observador 2 iniciada - recibe los últimos 2 valores
Observador 2: 2  // ← Valor pasado almacenado en búfer
Observador 2: 3  // ← Valor pasado almacenado en búfer
Source: 4
Observador 1: 4
Observador 2: 4
```

**Puntos Importantes**:
- Los suscriptores retrasados pueden recibir inmediatamente valores pasados almacenados en búfer
- Se recuerdan valores hasta el tamaño del búfer (2 en este ejemplo)

## 💡 Sintaxis de shareReplay()

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### Parámetros

| Parámetro | Tipo | Descripción | Predeterminado |
|-----------|---|------|----------|
| `bufferSize` | `number` | Número de valores a almacenar en búfer | `Infinity` |
| `windowTime` | `number` | Período de validez del búfer (milisegundos) | `Infinity` |
| `scheduler` | `SchedulerLike` | Scheduler para control de timing | - |

### Objeto de Configuración (RxJS 7+)

```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // Desuscribirse cuando el conteo de suscriptores llega a cero
  scheduler?: SchedulerLike;
}
```

## 📊 Diferencia Entre share y shareReplay

### Comportamiento de share()

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observador 1: ${value}`));

setTimeout(() => {
  console.log('Suscripción del Observador 2 iniciada');
  source$.subscribe(value => console.log(`Observador 2: ${value}`));
}, 1500);
```

**Resultado de Ejecución**:
```
Source: 0
Observador 1: 0
Source: 1
Observador 1: 1
Suscripción del Observador 2 iniciada
Source: 2
Observador 1: 2
Observador 2: 2  // ← No puede recibir valores pasados (0, 1)
```

### Comportamiento de shareReplay()

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source: ${value}`)),
  shareReplay(2) // Almacenar en búfer los últimos 2 valores
);

source$.subscribe(value => console.log(`Observador 1: ${value}`));

setTimeout(() => {
  console.log('Suscripción del Observador 2 iniciada');
  source$.subscribe(value => console.log(`Observador 2: ${value}`));
}, 1500);
```

**Resultado de Ejecución**:
```
Source: 0
Observador 1: 0
Source: 1
Observador 1: 1
Suscripción del Observador 2 iniciada
Observador 2: 0  // ← Valor pasado almacenado en búfer
Observador 2: 1  // ← Valor pasado almacenado en búfer
Source: 2
Observador 1: 2
Observador 2: 2
```

## 💼 Casos de Uso Prácticos

### 1. Almacenamiento en Caché de Respuestas API

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
  // Cachear información de usuario
  private userCache$ = ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
    tap(() => console.log('Solicitud API ejecutada')),
    shareReplay(1) // Cachear permanentemente el último 1 valor
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// Primer componente
userService.getUser().subscribe(user => {
  console.log('Componente 1:', user);
});

// Otro componente después de 2 segundos
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('Componente 2:', user); // ← Recuperado del caché, sin solicitud API
  });
}, 2000);
```

**Resultado de Ejecución**:
```
Solicitud API ejecutada
Componente 1: { id: 1, name: "John" }
Componente 2: { id: 1, name: "John" }  // ← Sin solicitud API
```

### 2. Compartir Información de Configuración

```typescript
import { of } from 'rxjs';
import { delay, shareReplay, tap } from 'rxjs';

// Obtener configuración de la aplicación (ejecutado solo una vez)
const appConfig$ = of({
  apiUrl: 'https://api.example.com',
  theme: 'dark',
  language: 'es'
}).pipe(
  delay(1000), // Simular carga
  tap(() => console.log('Configuración cargada')),
  shareReplay(1)
);

// Usar configuración en múltiples servicios
appConfig$.subscribe(config => console.log('Servicio A:', config.apiUrl));
appConfig$.subscribe(config => console.log('Servicio B:', config.theme));
appConfig$.subscribe(config => console.log('Servicio C:', config.language));
```

**Resultado de Ejecución**:
```
Configuración cargada
Servicio A: https://api.example.com
Servicio B: dark
Servicio C: es
```

### 3. Caché Limitado por Tiempo

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, tap } from 'rxjs';

// Cachear solo por 5 segundos (usando datos TODO como ejemplo)
const todoData$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1').pipe(
  tap(() => console.log('Datos TODO recuperados')),
  shareReplay({
    bufferSize: 1,
    windowTime: 5000, // Válido por 5 segundos
    refCount: true    // Desuscribirse cuando el conteo de suscriptores llega a cero
  })
);

// Primera suscripción
todoData$.subscribe(data => console.log('Obtener 1:', data));

// Después de 3 segundos (caché válido)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Obtener 2:', data)); // Del caché
}, 3000);

// Después de 6 segundos (caché expirado)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Obtener 3:', data)); // Nueva solicitud
}, 6000);
```

## ⚠️ Cuidado con las Fugas de Memoria

`shareReplay()` mantiene valores en un búfer, lo que puede causar fugas de memoria si no se gestiona adecuadamente.

### Código Problemático

```typescript
// ❌ Riesgo de fuga de memoria
const infiniteStream$ = interval(1000).pipe(
  shareReplay() // bufferSize no especificado = Infinity
);

// Este flujo continúa acumulando valores para siempre
```

### Contramedidas Recomendadas

```typescript
// ✅ Limitar tamaño del búfer
const safeStream$ = interval(1000).pipe(
  shareReplay(1) // Mantener solo el último 1
);

// ✅ Usar refCount
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    refCount: true // Limpiar búfer cuando el conteo de suscriptores llega a cero
  })
);

// ✅ Establecer límite de tiempo
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    windowTime: 10000 // Expira en 10 segundos
  })
);
```

## 🎯 Elegir Tamaño de Búfer

| Tamaño de Búfer | Caso de Uso | Ejemplo |
|--------------|-----------|---|
| `1` | Solo se necesita el último estado | Info de usuario actual, configuración |
| `3-5` | Se necesita historial reciente | Historial de chat, historial de notificaciones |
| `Infinity` | Se necesita todo el historial | Logs, pistas de auditoría (usar con precaución) |

## 🔄 Operadores Relacionados

- **[share()](/es/guide/operators/multicasting/share)** - Multicast simple (sin almacenamiento en búfer)
- **[publish()](/es/guide/subjects/multicasting)** - Control de multicast de bajo nivel
- **[ReplaySubject](/es/guide/subjects/types-of-subject)** - Subject que forma la base de shareReplay

## Resumen

El operador `shareReplay()`:
- Almacena en búfer valores pasados y los proporciona a suscriptores retrasados
- Ideal para almacenamiento en caché de respuestas API
- Requiere atención a las fugas de memoria
- Se puede usar de manera segura con `refCount` y `windowTime`

Cuando se necesita compartición de estado o almacenamiento en caché, `shareReplay()` es una herramienta muy poderosa, pero es importante establecer configuraciones apropiadas de tamaño de búfer y expiración.

## 🔗 Secciones Relacionadas

- **[Errores Comunes y Soluciones](/es/guide/anti-patterns/common-mistakes#4-misuse-of-sharereplay)** - Uso apropiado de shareReplay y contramedidas contra fugas de memoria
- **[share()](/es/guide/operators/multicasting/share)** - Multicast simple
- **[ReplaySubject](/es/guide/subjects/types-of-subject)** - Subject que forma la base de shareReplay
