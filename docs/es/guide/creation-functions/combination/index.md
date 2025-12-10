---
description: Esta sección describe las Funciones de Creación que combinan múltiples Observables en uno, y enseña cómo usar concat, merge, combineLatest, zip y forkJoin, así como ejemplos prácticos.
---

# Funciones de Creación de Combinación

Estas son las principales Funciones de Creación para combinar múltiples Observables en un solo Observable.

## ¿Qué son las Funciones de Creación de Combinación?

Las Funciones de Creación de Combinación toman múltiples Observables y los combinan en un único flujo Observable. El momento y el orden en que se emiten los valores depende del método de combinación.

La siguiente tabla muestra las características de cada Función de Creación y cómo usarlas.

## Principales Funciones de Creación de Combinación

| Función | Descripción | Casos de Uso |
|----------|------|-------------|
| **[concat](/es/guide/creation-functions/combination/concat)** | Combinación secuencial (la siguiente comienza después de que la anterior complete) | Procesamiento paso a paso |
| **[merge](/es/guide/creation-functions/combination/merge)** | Combinación paralela (suscribe simultáneamente, salida en orden de emisión) | Integración de múltiples eventos |
| **[combineLatest](/es/guide/creation-functions/combination/combineLatest)** | Combina los valores más recientes | Sincronización de entradas de formulario |
| **[zip](/es/guide/creation-functions/combination/zip)** | Empareja valores correspondientes | Emparejar solicitudes con respuestas |
| **[forkJoin](/es/guide/creation-functions/combination/forkJoin)** | Espera a que todos completen y combina los valores finales | Esperar a que completen llamadas API paralelas |

## Criterios de Uso

La selección de Funciones de Creación de Combinación se determina desde las siguientes perspectivas:

### 1. Momento de Ejecución

- **Ejecución secuencial**: `concat` - Inicia la siguiente después de que el Observable anterior complete
- **Ejecución paralela**: `merge`, `combineLatest`, `zip`, `forkJoin` - Se suscribe a todos los Observables simultáneamente

### 2. Cómo Emitir Valores

- **Emitir todos los valores**: `concat`, `merge` - Emite todos los valores de cada Observable
- **Combinar valores más recientes**: `combineLatest` - Combina todos los valores más recientes cada vez que uno emite
- **Emparejar valores correspondientes**: `zip` - Empareja valores de posiciones correspondientes en cada Observable y emite
- **Solo valores finales**: `forkJoin` - Emite cada valor final como un array cuando todos los Observables completan

### 3. Momento de Completación

- **Después de que todos completen**: `concat`, `forkJoin` - Espera hasta que todos los Observables hayan completado
- **Completa con el flujo más corto**: `zip` - Completa cuando cualquiera completa, ya que los valores restantes no pueden formar pares
- **No completa**: `merge`, `combineLatest` - Si uno completa mientras el otro continúa, no completará

## Convertir Cold a Hot

Como se muestra en la tabla anterior, **todas las Funciones de Creación de Combinación generan Cold Observables**. Cada suscripción inicia una ejecución independiente.

Sin embargo, puedes **convertir un Cold Observable a Hot Observable** usando un operador de multicast (`share()`, `shareReplay()`, `publish()`, etc.).

### Ejemplo Práctico: Compartir Solicitudes HTTP

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - Solicitudes HTTP independientes para cada suscripción
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Fuente A'), take(3)),
  interval(1500).pipe(map(() => 'Fuente B'), take(2))
);

coldApi$.subscribe(val => console.log('Suscriptor 1:', val));
coldApi$.subscribe(val => console.log('Suscriptor 2:', val));
// → Cada suscriptor ejecuta intervalos independientes (2x solicitudes)

// 🔥 Hot - Comparte ejecución entre suscriptores
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Fuente A'), take(3)),
  interval(1500).pipe(map(() => 'Fuente B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('Suscriptor 1:', val));
hotApi$.subscribe(val => console.log('Suscriptor 2:', val));
// → Comparte un intervalo (solicitudes solo una vez)
```

> [!TIP]
> **Casos donde se requiere conversión a Hot**:
> - Múltiples componentes comparten los mismos resultados API
> - Usar `forkJoin` para usar los resultados de solicitudes paralelas en múltiples lugares
> - Gestionar estado con `combineLatest` y distribuir a múltiples suscriptores
>
> Para más información, consulta [Creación Básica - Convertir Cold a Hot](/es/guide/creation-functions/basic/#convertir-cold-a-hot).

## Correspondencia con Pipeable Operator

Para las Funciones de Creación de Combinación, existe un Pipeable Operator correspondiente. Cuando se usa en un pipeline, se utiliza el operador tipo `~With`.

| Función de Creación | Pipeable Operator |
|-------------------|-------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## Próximos Pasos

Para aprender el comportamiento detallado y ejemplos prácticos de cada Función de Creación, haz clic en los enlaces de la tabla anterior.

Además, aprende sobre [Funciones de Creación de Selección/Partición](/es/guide/creation-functions/selection/) y [Funciones de Creación Condicionales](/es/guide/creation-functions/conditional/), para entender el panorama completo de las Funciones de Creación.
