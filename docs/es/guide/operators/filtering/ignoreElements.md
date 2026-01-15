---
description: El operador ignoreElements es un operador de filtrado de RxJS que ignora todos los valores y pasa solo notificaciones de completación y error. Es útil cuando se espera la completación del proceso.
titleTemplate: ':title'
---

# ignoreElements - Solo Completación

El operador `ignoreElements` ignora **todos los valores** emitidos del Observable fuente y pasa **solo notificaciones de completación y error** aguas abajo.

## 🔰 Sintaxis Básica y Uso

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valor:', value), // No se llama
  complete: () => console.log('Completado')
});
// Salida: Completado
```

**Flujo de operación**:
1. 1, 2, 3, 4, 5 se ignoran todos
2. Solo la notificación de completación se propaga aguas abajo

[🌐 Documentación Oficial de RxJS - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Patrones de Uso Típicos

- **Esperar completación del proceso**: Cuando los valores son innecesarios y solo se necesita la completación
- **Ejecutar solo efectos secundarios**: Ejecutar efectos secundarios con tap e ignorar valores
- **Manejo de errores**: Cuando deseas capturar solo errores
- **Sincronización de secuencias**: Esperar completación de múltiples procesos

## 🆚 Comparación con Operadores Similares

### ignoreElements vs filter(() => false) vs take(0)

| Operador | Procesamiento de Valores | Notificación de Completación | Caso de Uso |
|:---|:---|:---|:---|
| `ignoreElements()` | Ignorar todos | Pasar | **Solo se necesita completación** (recomendado) |
| `filter(() => false)` | Filtrar todos | Pasar | Filtrado condicional (coincidentemente todos excluidos) |
| `take(0)` | Completar inmediatamente | Pasar | Quiere completar inmediatamente |

**Recomendado**: Usar `ignoreElements()` cuando se ignoran intencionalmente todos los valores. Hace clara la intención del código.

## 📚 Operadores Relacionados

- **[filter](/es/guide/operators/filtering/filter)** - Filtrar valores basándose en condiciones
- **[take](/es/guide/operators/filtering/take)** - Obtener solo primeros N valores
- **[skip](/es/guide/operators/filtering/skip)** - Omitir primeros N valores
- **[tap](https://rxjs.dev/api/operators/tap)** - Ejecutar efectos secundarios (documentación oficial)

## Resumen

El operador `ignoreElements` ignora todos los valores y pasa solo completación y error.

- ✅ Ideal cuando solo se necesita notificación de completación
- ✅ Se ejecutan efectos secundarios (tap)
- ✅ También pasa notificaciones de error
- ✅ Intención más clara que `filter(() => false)`
- ⚠️ No se completa con Observables infinitos
- ⚠️ El tipo de valor de retorno es `Observable<never>`
- ⚠️ Los valores se ignoran completamente pero se ejecutan efectos secundarios
