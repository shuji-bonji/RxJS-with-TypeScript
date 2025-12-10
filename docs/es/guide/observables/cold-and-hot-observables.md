---
description: Este artículo detalla las diferencias entre Cold Observable y Hot Observable. Se presentan ejemplos de aplicación práctica, incluyendo la independencia del flujo de datos por suscripción, cómo convertir Cold a Hot usando share y shareReplay, y el almacenamiento en caché de solicitudes de API.
---
# Cold Observable y Hot Observable

Uno de los conceptos clave al usar RxJS es la distinción entre "Cold Observable" y "Hot Observable". Entender esta distinción es esencial para aprender cómo usar Observable de manera eficiente.

## Por Qué es Importante Entender Cold/Hot

Si no entiende la distinción Cold/Hot, enfrentará los siguientes problemas:

- **Ejecución Duplicada No Intencionada** - Las llamadas de API se ejecutan múltiples veces
- **Fugas de memoria** - Las suscripciones no se gestionan apropiadamente
- **Problemas de rendimiento** - El procesamiento innecesario se repite
- **Inconsistencias de datos** - No se reciben los datos esperados

## Diferencias Cold vs Hot (Gráfico Comparativo)

Primero, obtengamos la visión general.

| Ítem de Comparación | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| **Ejecución sin suscripción** | No se ejecuta (se ejecuta solo al suscribirse) | Se ejecuta (fluye valores incluso sin subscribe) |
| **Momento de publicación de datos** | Comienza cuando se llama a `subscribe()` | Comienza en el momento del publicador (independiente de la suscripción) |
| **Reutilización de ejecución** | Se ejecuta de nuevo cada vez | Comparte el flujo existente con múltiples suscriptores |
| **Consistencia de datos** | Cada suscripción recibe valores independientes | No puede recibir valores pasados si se suscribe a mitad de flujo |
| **Principales casos de uso** | Solicitudes HTTP, procesamiento asíncrono | Eventos de UI, WebSocket, comunicación en tiempo real |
| **Escenarios de uso** | Cuando cada proceso es independiente | Compartir estado, difusión de eventos |

**Criterios:** ¿Debería el procesamiento ejecutarse nuevamente para cada suscriptor? ¿O debería compartirse el flujo?

## Criterios de Decisión Cold vs. Hot

Los siguientes criterios se pueden usar para determinar si un Observable es realmente Cold o Hot:

| Punto de Decisión | Cold | Hot |
|-------------|------|-----|
| **¿Se vuelve a ejecutar la lógica de ejecución para cada suscripción?** | ✅ Se vuelve a ejecutar cada vez | ❌ Compartir ejecución |
| **¿Fluyen los datos antes de la suscripción?** | ❌ Espera hasta suscribirse | ✅ Fluye independientemente de la suscripción |
| **¿Múltiples suscripciones reciben los mismos datos?** | ❌ Datos independientes | ✅ Compartir los mismos datos |

### Formas Prácticas de Distinguir

La siguiente prueba puede determinar fácilmente:

```typescript
const observable$ = /* Observable a examinar */;

observable$.subscribe(/* suscripción 1 */);
observable$.subscribe(/* suscripción 2 */);

// ✅ Cold: console.log dentro de Observable se ejecuta dos veces
//         (la lógica de ejecución se vuelve a ejecutar para cada suscripción)
// ✅ Hot:  console.log dentro de Observable se ejecuta solo una vez
//         (la ejecución se comparte)
```

**Ejemplo Concreto:**

```typescript
import { Observable, Subject } from 'rxjs';

// Cold Observable
const cold$ = new Observable(subscriber => {
  console.log('Cold: Ejecución iniciada');
  subscriber.next(Math.random());
});

cold$.subscribe(v => console.log('Suscripción 1:', v));
cold$.subscribe(v => console.log('Suscripción 2:', v));
// Salida:
// Cold: Ejecución iniciada  ← 1ra vez
// Suscripción 1: 0.123...
// Cold: Ejecución iniciada  ← 2da vez (re-ejecutado)
// Suscripción 2: 0.456...

// Hot Observable
const hot$ = new Subject();

hot$.subscribe(v => console.log('Suscripción 1:', v));
hot$.subscribe(v => console.log('Suscripción 2:', v));
hot$.next(1); // Datos publicados solo una vez
// Salida:
// Suscripción 1: 1
// Suscripción 2: 1  ← Comparte los mismos datos
```

## Tabla de Clasificación Cold/Hot por Creation Function

Esta tabla clasifica Cold/Hot para todas las Creation Functions principales. Esto le permite ver de un vistazo qué función produce qué Observable.

| Categoría | Creation Function | Cold/Hot | Notas |
|---------|-------------------|----------|------|
| **Creación básica** | `of()` | ❄️ Cold | Re-publica valores para cada suscripción |
| | `from()` | ❄️ Cold | Re-ejecuta array/Promise para cada suscripción |
| | `fromEvent()` | ❄️ Cold | Agrega escuchador independiente para cada suscripción [^fromEvent] |
| | `interval()` | ❄️ Cold | Temporizador independiente para cada suscripción |
| | `timer()` | ❄️ Cold | Temporizador independiente para cada suscripción |
| **Generación de bucle** | `range()` | ❄️ Cold | Regenera rango para cada suscripción |
| | `generate()` | ❄️ Cold | Re-ejecuta bucle para cada suscripción |
| **Comunicación HTTP** | `ajax()` | ❄️ Cold | Nueva solicitud HTTP para cada suscripción |
| | `fromFetch()` | ❄️ Cold | Nueva solicitud Fetch para cada suscripción |
| **Combinación** | `concat()` | ❄️ Cold | Hereda naturaleza de Observables fuente [^combination] |
| | `merge()` | ❄️ Cold | Hereda naturaleza de Observables fuente [^combination] |
| | `combineLatest()` | ❄️ Cold | Hereda naturaleza de Observables fuente [^combination] |
| | `zip()` | ❄️ Cold | Hereda naturaleza de Observables fuente [^combination] |
| | `forkJoin()` | ❄️ Cold | Hereda naturaleza de Observables fuente [^combination] |
| **Selección/Partición** | `race()` | ❄️ Cold | Hereda naturaleza de Observables fuente [^combination] |
| | `partition()` | ❄️ Cold | Hereda naturaleza de Observables fuente [^combination] |
| **Condicional** | `iif()` | ❄️ Cold | Hereda naturaleza de Observable condicionalmente seleccionado |
| | `defer()` | ❄️ Cold | Ejecuta función factory para cada suscripción |
| **Control** | `scheduled()` | ❄️ Cold | Hereda naturaleza de Observable fuente |
| | `using()` | ❄️ Cold | Crea recurso para cada suscripción |
| **Familia Subject** | `new Subject()` | 🔥 Hot | Siempre Hot |
| | `new BehaviorSubject()` | 🔥 Hot | Siempre Hot |
| | `new ReplaySubject()` | 🔥 Hot | Siempre Hot |
| | `new AsyncSubject()` | 🔥 Hot | Siempre Hot |
| **WebSocket** | `webSocket()` | 🔥 Hot | Comparte conexión WebSocket |

[^fromEvent]: `fromEvent()` es Cold porque agrega un escuchador de eventos independiente para cada suscripción. Sin embargo, el evento en sí ocurre independientemente de la suscripción, por lo que se malinterpreta fácilmente como Hot.

[^combination]: Las Creation Functions de combinación son Cold si el Observable fuente es Cold, y Hot si es Hot. Usualmente, se combinan Observables Cold.

> [!IMPORTANT] Principio Clave
> **Casi todas las Creation Functions generan Cold.**
> Solo las siguientes generan Hot:
> - Familia Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject)
> - webSocket()

## Cold Observable

### Características

- **Se crea un nuevo flujo de datos cada vez que se realiza una suscripción**
- **La publicación de datos no comienza hasta que se suscribe (ejecución perezosa)**
- **Todos los suscriptores reciben todos los datos desde el principio del Observable**

Cold Observable crea un nuevo contexto de ejecución cada vez que se suscribe.
Esto es apropiado para solicitudes HTTP, procesamiento asíncrono y otras situaciones donde se requiere nuevo procesamiento cada vez.

### Ejemplo de Código

```typescript
import { Observable } from 'rxjs';

// Ejemplo de Cold Observable
const cold$ = new Observable<number>(subscriber => {
  console.log('Creación de fuente de datos - nueva suscripción');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.complete();
});

// Primera suscripción
console.log('--- Primera suscripción ---');
cold$.subscribe(value => console.log('Suscriptor 1:', value));

// Segunda suscripción (se generan datos diferentes)
console.log('--- Segunda suscripción ---');
cold$.subscribe(value => console.log('Suscriptor 2:', value));
```

#### Salida
```sh
--- Primera suscripción ---
Creación de fuente de datos - nueva suscripción
Suscriptor 1: 0.259632...
--- Segunda suscripción ---
Creación de fuente de datos - nueva suscripción  ← Re-ejecutado
Suscriptor 2: 0.744322...  ← Valor diferente
```

> [!TIP] Punto Importante
> Cada suscripción ejecuta "Creación de fuente de datos" y genera valores diferentes.

### Observables Cold Comunes (Cómo Identificar)

Los siguientes Observables son usualmente Cold:

```typescript
import { of, from, interval, timer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Creation Functions
of(1, 2, 3)                    // Cold
from([1, 2, 3])                // Cold
from(fetch('/api/data'))       // Cold

// Operadores de tiempo
interval(1000)                 // Cold
timer(1000)                    // Cold

// Solicitudes HTTP
ajax('/api/users')             // Cold
```

> [!TIP] Regla
> Creation Functions, operadores de tiempo y solicitudes HTTP son básicamente Cold

## Hot Observable

### Características

- **Fluye valores incluso si no está suscrito (se ejecuta con o sin suscripción)**
- **Solo recibe datos desde el punto de inicio de suscripción en adelante**
- **Una fuente de datos es compartida por múltiples suscriptores**

Con Hot Observable, el momento de publicación de flujo es independiente de la suscripción, y los suscriptores se unen a mitad del flujo.

### Ejemplo de Código

```typescript
import { Subject } from 'rxjs';

// Ejemplo de Hot Observable (usando Subject)
const hot$ = new Subject<number>();

// Primera suscripción
console.log('--- Suscriptor 1 comienza ---');
hot$.subscribe(value => console.log('Suscriptor 1:', value));

// Publicar datos
hot$.next(1);
hot$.next(2);

// Segunda suscripción (suscripción tardía)
console.log('--- Suscriptor 2 comienza ---');
hot$.subscribe(value => console.log('Suscriptor 2:', value));

// Publicar más datos
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### Salida
```sh
--- Suscriptor 1 comienza ---
Suscriptor 1: 1
Suscriptor 1: 2
--- Suscriptor 2 comienza ---
Suscriptor 1: 3
Suscriptor 2: 3  ← Suscripción 2 se une desde 3 (no puede recibir 1, 2)
Suscriptor 1: 4
Suscriptor 2: 4
```

> [!TIP] Punto Importante
> Suscriptor 2 se unió a mitad de flujo y no puede recibir valores pasados (1, 2).

### Observables Hot Comunes (Cómo Identificar)

Los siguientes Observables son siempre Hot:

```typescript
import { Subject, BehaviorSubject, ReplaySubject } from 'rxjs';
import { webSocket } from 'rxjs/webSocket';

// Familia Subject (siempre Hot)
new Subject()                  // Hot
new BehaviorSubject(0)         // Hot
new ReplaySubject(1)           // Hot

// WebSocket (siempre Hot)
webSocket('ws://localhost:8080') // Hot
```

> [!TIP] Regla
> **Solo la familia Subject y webSocket() generan Hot**

> [!WARNING] fromEvent() es Cold
> `fromEvent(button, 'click')` a menudo se malinterpreta como Hot, pero en realidad es **Cold**. Agrega un escuchador de eventos independiente para cada suscripción. El evento en sí ocurre independientemente de la suscripción, pero cada suscriptor tiene un escuchador independiente.

## Cómo Convertir Cold Observable a Hot

En RxJS, los siguientes son los medios principales para convertir Cold Observable a Hot:

- `share()` - Conversión hot simple (recomendado)
- `shareReplay()` - Almacenar en caché valores pasados y convertir a hot
- ~~`multicast()`~~ - Obsoleto (obsoleto en RxJS v7, eliminado en v8)

### Operador share()

`share()` es la forma más común de convertir un Cold Observable a un Hot Observable.

```typescript
import { interval } from 'rxjs';
import { share, take } from 'rxjs';

// Simular llamada HTTP
const makeHttpRequest = () => {
  console.log('¡Llamada HTTP ejecutada!');
  return interval(1000).pipe(take(3));
};

// ❌ Cold Observable (sin compartir)
const cold$ = makeHttpRequest();

cold$.subscribe(val => console.log('Suscriptor 1:', val));
cold$.subscribe(val => console.log('Suscriptor 2:', val));
// → Llamada HTTP ejecutada dos veces

// ✅ Hot Observable (usando share)
const shared$ = makeHttpRequest().pipe(share());

shared$.subscribe(val => console.log('Suscriptor compartido 1:', val));
shared$.subscribe(val => console.log('Suscriptor compartido 2:', val));
// → Llamada HTTP ejecutada solo una vez, el resultado se comparte
```

**Salida (Cold):**
```sh
¡Llamada HTTP ejecutada!  ← 1ra vez
Suscriptor 1: 0
¡Llamada HTTP ejecutada!  ← 2da vez (¡duplicado!)
Suscriptor 2: 0
...
```

**Salida (Hot):**
```sh
¡Llamada HTTP ejecutada!  ← Solo una vez
Suscriptor compartido 1: 0
Suscriptor compartido 2: 0  ← Comparte el mismo flujo
...
```

> [!NOTE] Casos de Uso
> - Usar los mismos resultados de API en múltiples componentes
> - Evitar efectos secundarios duplicados (ej., llamadas HTTP)

### Operador shareReplay()

`shareReplay()` es una extensión de `share()` que **almacena en caché** valores pasados y los reproduce para nuevos suscriptores.

```typescript
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

const request$ = interval(1000).pipe(
  take(3),
  shareReplay(2)  // Almacenar en caché los últimos 2 valores
);

// Primera suscripción
request$.subscribe(val => console.log('Suscriptor 1:', val));

// Segunda suscripción después de 3.5 segundos (después de completar el flujo)
setTimeout(() => {
  console.log('--- Suscriptor 2 comienza (después de finalización) ---');
  request$.subscribe(val => console.log('Suscriptor 2:', val));
}, 3500);
```

#### Salida
```sh
Suscriptor 1: 0
Suscriptor 1: 1
Suscriptor 1: 2
--- Suscriptor 2 comienza (después de finalización) ---
Suscriptor 2: 1  ← Valor en caché (últimos 2)
Suscriptor 2: 2  ← Valor en caché
```

> [!NOTE] Casos de Uso
> - Almacenar en caché resultados de API
> - Compartir estado inicial (almacenar en caché solo el último)
> - Proporcionar datos históricos a suscriptores tardíos

> [!WARNING] Notas sobre shareReplay
> `shareReplay()` continúa manteniendo el caché incluso cuando las suscripciones llegan a 0, lo que puede causar fugas de memoria. Consulte [Capítulo 10: Mal uso de shareReplay](/es/guide/anti-patterns/common-mistakes#4-sharereplay-misuse) para más información.

### Acerca de multicast()

> [!NOTE]
> `multicast()` es flexible, pero fue obsoleto en RxJS v7 y eliminado en v8. Use `share()` o `shareReplay()` ahora. Consulte [descripción del operador share()](/es/guide/operators/multicasting/share) para más información.

## Ejemplo Práctico: Servicio de Caché de API

Patrón común en aplicaciones reales: múltiples componentes necesitan los mismos datos de API.

```typescript
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, delay, tap } from 'rxjs';

// Servicio de caché simple
class UserService {
  private cache$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // Devolver caché si existe
    if (this.cache$) {
      console.log('Devolviendo desde caché');
      return this.cache$;
    }

    // Crear nueva solicitud y almacenar en caché
    console.log('Ejecutando nueva solicitud');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      catchError(err => {
        this.cache$ = null;  // Limpiar caché en caso de error
        return throwError(() => err);
      }),
      shareReplay(1)  // Almacenar en caché el último resultado
    );

    return this.cache$;
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    // Simular solicitud de API real
    return of([
      { id: 1, name: 'Taro Yamada' },
      { id: 2, name: 'Hanako Sato' }
    ]).pipe(
      delay(1000),
      tap(() => console.log('Datos recibidos de API'))
    );
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('Caché limpiado');
  }
}

interface User {
  id: number;
  name: string;
}

// Ejemplo de uso
const userService = new UserService();

// Componente 1: Solicitar datos
userService.getUsers().subscribe(users =>
  console.log('Componente 1:', users)
);

// Componente 2: Solicitar datos después de 2 segundos
setTimeout(() => {
  userService.getUsers().subscribe(users =>
    console.log('Componente 2:', users)
  );
}, 2000);

// Limpiar caché y solicitar nuevamente
setTimeout(() => {
  userService.clearCache();
  userService.getUsers().subscribe(users =>
    console.log('Componente 3:', users)
  );
}, 4000);
```

#### Salida
```sh
Ejecutando nueva solicitud
Datos recibidos de API
Componente 1: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Devolviendo desde caché  ← Sin llamada de API
Componente 2: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Caché limpiado
Ejecutando nueva solicitud  ← Llamada de API nuevamente
Datos recibidos de API
Componente 3: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
```

**Puntos:**
- Almacenar en caché la última respuesta con `shareReplay(1)`
- Múltiples componentes comparten datos (solo una llamada de API)
- Descartar caché apropiadamente en caso de error o limpiar

## Cuándo Usar

<div class="comparison-cards">

::: tip Cold
#### Cuándo usar
- Cuando cada suscriptor necesita su propio conjunto de datos
- Cuando se representa un proceso o acción recién iniciados
- Cuando los efectos secundarios duplicados no son un problema

#### Ejemplos
- Enviar una nueva solicitud POST para cada envío de formulario
- Se necesita un temporizador diferente para cada usuario
- Ejecutar cálculo independiente para cada suscripción
:::

::: tip Hot
#### Cuándo usar
- Cuando se comparten datos entre múltiples componentes
- Cuando se desea ahorrar recursos (ej., reducir el número de llamadas HTTP)
- Cuando se representan flujos de eventos
- Gestión de estado o comunicación entre servicios

#### Ejemplos
- Información de configuración compartida en toda la aplicación
- Estado de inicio de sesión del usuario
- Mensajes en tiempo real (WebSocket)
- Eventos DOM (clic, desplazamiento, etc.)
:::

</div>

## Resumen

Entender y usar apropiadamente Cold Observable y Hot Observable es una habilidad importante para construir aplicaciones RxJS eficientes.

::: tip Puntos Clave
- **Cold Observable**: Un flujo que comienza a ejecutarse solo después de suscribirse (ejecución independiente por suscripción)
- **Hot Observable**: Comparte un flujo que ya se está ejecutando (misma ejecución para múltiples suscripciones)
- **share()**: Forma más fácil de convertir Cold a Hot
- **shareReplay()**: Almacenar en caché valores pasados y convertir a Hot (útil para compartir resultados de API)
:::

::: tip Criterios de Decisión de Diseño
- ¿Necesita compartir datos entre múltiples suscriptores?
- ¿Es necesario almacenar en caché valores pasados y proporcionarlos a nuevos suscriptores?
- ¿Cómo se gestionarán los efectos secundarios duplicados (ej., solicitudes HTTP)?
:::

Basándose en estas consideraciones, seleccionar el tipo apropiado de Observable y operador le ayudará a construir una aplicación reactiva eficiente y robusta.

## Secciones Relacionadas

- **[operador share()](/es/guide/operators/multicasting/share)** - Explicación detallada de share()
- **[Mal uso de shareReplay](/es/guide/anti-patterns/common-mistakes#4-sharereplay-misuse)** - Errores comunes y soluciones
- **[Subject](/es/guide/subjects/what-is-subject)** - Entendiendo Hot Subjects

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
