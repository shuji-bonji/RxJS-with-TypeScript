---
description: "Explicamos las Funciones de Creación que seleccionan o crean Observables basándose en condiciones (iif y defer). iif realiza bifurcaciones condicionales como un operador ternario, defer realiza evaluación diferida al momento de la suscripción. Presentamos cómo usarlas, implementación segura de tipos en TypeScript y casos de uso prácticos."
---

# Funciones de Creación Condicionales

Funciones de Creación que seleccionan Observables basándose en condiciones o generan Observables dinámicamente al momento de la suscripción.

## ¿Qué son las Funciones de Creación Condicionales?

Las Funciones de Creación Condicionales tienen los siguientes roles:

- **Selección por condición**: Seleccionar diferentes Observables según la condición
- **Generación diferida**: Generar Observables dinámicamente al momento de la suscripción

A diferencia de otras Funciones de Creación que crean y combinan Observables estáticamente, estas pueden cambiar su comportamiento basándose en **condiciones o estados en tiempo de ejecución**.

> [!NOTE]
> `iif` y `defer` anteriormente se clasificaban como "operadores condicionales", pero son **Funciones de Creación** (funciones de creación de Observable), no Pipeable Operators.

## Principales Funciones de Creación Condicionales

| Función | Descripción | Caso de Uso |
|----------|------|-------------|
| **[iif](/es/guide/creation-functions/conditional/iif)** | Elige entre dos Observables según la condición | Bifurcación de procesamiento por estado de inicio de sesión |
| **[defer](/es/guide/creation-functions/conditional/defer)** | Generación diferida de Observable al momento de suscripción | Creación dinámica de Observable |

## Criterios de Selección

### iif - Bifurcación de dos opciones por condición

`iif` selecciona entre dos Observables según el resultado de la función de condición. La condición se evalúa **al momento de la suscripción**.

**Sintaxis**:
```typescript
iif(
  () => condition,  // Función de condición (evaluada al suscribir)
  trueObservable,   // Observable para el caso true
  falseObservable   // Observable para el caso false
)
```

**Casos de uso**:
- Bifurcación de procesamiento según estado de inicio de sesión
- Cambio de procesamiento según existencia de caché
- Cambio de comportamiento por variables de entorno

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Datos autenticados'),
  of('Datos públicos')
);

data$.subscribe(console.log);
// Salida: 'Datos autenticados' o 'Datos públicos' (según la condición al suscribir)
```

### defer - Generación diferida al momento de suscripción

`defer` genera un Observable cada vez que ocurre una suscripción. Esto permite cambiar el comportamiento del Observable basándose en el estado al momento de la suscripción.

**Sintaxis**:
```typescript
defer(() => {
  // Se ejecuta al momento de la suscripción
  return someObservable;
})
```

**Casos de uso**:
- Generación de Observable que refleja el estado más reciente al suscribir
- Generar diferentes valores aleatorios cada vez
- Ejecutar diferentes procesamientos por cada suscripción

```typescript
import { defer, of } from 'rxjs';

// Obtener la hora actual al suscribir
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Primero:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Segundo:', time));
}, 2000);

// Salida:
// Primero: 2024-10-21T01:00:00.000Z
// Segundo: 2024-10-21T01:00:01.000Z
// ※ Se muestran diferentes horas porque el momento de suscripción es diferente
```

## Diferencias entre iif y defer

| Característica | iif | defer |
|------|-----|-------|
| **Opciones** | Selección entre 2 Observables | Generación de Observable arbitrario |
| **Momento de evaluación** | Evalúa condición al suscribir | Ejecuta función al suscribir |
| **Uso** | Bifurcación condicional | Generación dinámica |

## Uso en Pipeline

Las Funciones de Creación Condicionales se pueden usar combinadas con otros operadores.

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// Obtener información de usuario desde ID de usuario
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // Verificar el caché más reciente al suscribir
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Conversión de Cold a Hot

Como se muestra en la tabla anterior, **todas las Funciones de Creación Condicionales generan Cold Observables**. Cada suscripción ejecuta la evaluación de condición o la función de generación.

Usando operadores de multicast (`share()`, `shareReplay()`, etc.), puedes convertir un Cold Observable en un Hot Observable.

### Ejemplo Práctico: Compartir Resultado de Bifurcación Condicional

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - Re-evalúa la condición por cada suscripción
const coldIif$ = iif(
  condition,
  of('La condición es true'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('Suscriptor 1:', val));
coldIif$.subscribe(val => console.log('Suscriptor 2:', val));
// → Cada suscriptor evalúa la condición independientemente (posibles resultados diferentes)

// 🔥 Hot - Comparte el resultado de evaluación de condición entre suscriptores
const hotIif$ = iif(
  condition,
  of('La condición es true'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('Suscriptor 1:', val));
hotIif$.subscribe(val => console.log('Suscriptor 2:', val));
// → La condición se evalúa solo una vez, el resultado se comparte
```

> [!TIP]
> Para más información, consulta [Creación Básica - Conversión de Cold a Hot](/es/guide/creation-functions/basic/#conversion-de-cold-a-hot).

## Próximos Pasos

Para aprender el comportamiento detallado y ejemplos prácticos de cada Función de Creación, haz clic en los enlaces de la tabla anterior.

Además, aprendiendo también [Funciones de Creación de Combinación](/es/guide/creation-functions/combination/) y [Funciones de Creación de Selección/División](/es/guide/creation-functions/selection/), podrás entender el panorama completo de las Funciones de Creación.
