---
description: "dematerialize est un opérateur utilitaire RxJS qui convertit les objets Notification en notifications normales (next, error, complete), en effectuant la transformation inverse de materialize. Idéal pour la restauration après le traitement des notifications, le filtrage ou la transformation des erreurs, la réorganisation ou la mise en mémoire tampon des notifications."
---

# dematerialize - Restauration des objets de notification

L'opérateur `dematerialize` **convertit les objets Notification en notifications normales (next, error, complete)**. Il effectue la transformation inverse de `materialize`, en restaurant les notifications converties en données dans leur format d'origine.

## 🔰 Syntaxe et comportement de base

Convertit un flux d'objets Notification en un flux normal.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // Conversion en objets Notification
    dematerialize()    // Restauration à l'original
  )
  .subscribe({
    next: v => console.log('Valeur :', v),
    complete: () => console.log('Complet')
  });
// Sortie :
// Valeur : 1
// Valeur : 2
// Valeur : 3
// Complet
```

[🌐 Documentation officielle RxJS - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 Cas d'utilisation typiques

- **Restauration après le traitement des notifications** : Restaurer le format d'origine après le traitement avec materialize
- **Filtrage des erreurs** : Exclure uniquement des erreurs spécifiques
- **Réorganisation des notifications** : Restauration après le tri des notifications en tant que données
- **Restauration après débogage** : Retour au fonctionnement normal après le traitement d'une sortie de journal

## 🧪 Exemple de code pratique : Filtrage sélectif des erreurs

Exemple où seules certaines erreurs sont exclues et le reste est traité normalement.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// Flux avec erreurs
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('Erreur à ignorer')),
  of(3, 4),
  throwError(() => new Error('Erreur critique')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // Filtrer uniquement "Erreur à ignorer"
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('à ignorer')) {
          console.log(`🔇 Ignoré : ${errorMessage}`);
          return false;  // Exclure cette erreur
        }
      }
      return true;
    }),
    dematerialize()  // Restaurer au format original
  )
  .subscribe({
    next: v => console.log(`✅ Valeur : ${v}`),
    error: err => console.log(`❌ Erreur : ${err.message}`),
    complete: () => console.log('Complet')
  });
// Sortie :
// ✅ Valeur : 1
// ✅ Valeur : 2
// 🔇 Ignoré : Erreur à ignorer
// ✅ Valeur : 3
// ✅ Valeur : 4
// ❌ Erreur : Erreur critique
```

## 🆚 Relation avec materialize

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // Conversion en Notification
    map(notification => {
      // Traitement en tant qu'objet Notification
      console.log('kind :', notification.kind);
      return notification;
    }),
    dematerialize()          // Restauration
  )
  .subscribe(v => console.log('Valeur :', v));
// Sortie :
// kind : N
// Valeur : 1
// kind : N
// Valeur : 2
// kind : N
// Valeur : 3
// kind : C
```

| Flux de traitement | Description |
|:---|:---|
| Flux original | Valeurs normales (next), erreurs (error), achèvement (complete) |
| ↓ `materialize()` | Flux d'objets Notification |
| Traitement intermédiaire | Manipulation, filtrage en tant que Notification |
| ↓ `dematerialize()` | Restauration en flux normal |
| Flux final | Valeurs, erreurs, achèvement normaux |

## ⚠️ Notes importantes

### 1. Les notifications d'erreur deviennent de vraies erreurs

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Chaque Observable converti avec materialize()
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('Erreur')).pipe(materialize()),
  of(2).pipe(materialize())  // Non exécuté après l'erreur
)
  .pipe(dematerialize())
  .subscribe({
    next: v => console.log('Valeur :', v),
    error: err => console.log('Erreur :', err.message)
  });
// Sortie :
// Valeur : 1
// Erreur : Erreur
```

### 2. La notification de completion termine le flux

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // Notification de completion
  of(3).pipe(materialize())   // Non exécuté après completion
)
  .pipe(dematerialize())
  .subscribe({
    next: v => console.log('Valeur :', v),
    complete: () => console.log('Complet')
  });
// Sortie :
// Valeur : 1
// Valeur : 2
// Complet
```

### 3. Objets Notification invalides

`dematerialize` attend des objets Notification corrects.

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ Passer des valeurs normales à dematerialize cause une erreur
of(1, 2, 3)
  .pipe(dematerialize())  // Ce ne sont pas des objets Notification
  .subscribe({
    next: console.log,
    error: err => console.error('Erreur :', err.message)
  });
// Une erreur se produit
```

## 📚 Opérateurs associés

- **[materialize](./materialize)** - Convertir les notifications en objets Notification
- **[catchError](/fr/guide/error-handling/retry-catch)** - Gestion des erreurs
- **[retry](./retry)** - Réessai en cas d'erreur

## ✅ Résumé

- `dematerialize` convertit les objets Notification en notifications normales
- ✅ Transformation inverse de `materialize`
- ✅ Restauration au format original après traitement des notifications
- ✅ Permet le filtrage ou la transformation des erreurs
- ✅ Utile pour la réorganisation ou la mise en mémoire tampon des notifications
- ⚠️ Les notifications d'erreur fonctionnent comme de vraies erreurs
- ⚠️ La notification de completion termine le flux
- ⚠️ Des objets Notification corrects sont requis
