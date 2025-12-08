---
description: "materialize est un opérateur utilitaire RxJS qui convertit les notifications Observable (next, error, complete) en objets Notification. Idéal pour traiter les erreurs comme des données, déboguer et enregistrer des notifications, enregistrer des méta-informations, et d'autres situations où vous voulez manipuler les notifications elles-mêmes. Le format d'origine peut être restauré avec dematerialize."
---

# materialize - Convertir les notifications en objets

L'opérateur `materialize` **convertit les notifications Observable (next, error, complete) en objets Notification**. Cela vous permet de traiter non seulement les valeurs mais aussi les erreurs et les achèvements comme des données.

## 🔰 Syntaxe et comportement de base

Convertit un flux normal en un flux d'objets Notification.

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// Sortie :
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

La propriété `kind` des objets Notification :
- `'N'` : next (émission de valeur)
- `'E'` : error (erreur)
- `'C'` : complete (achèvement)

[🌐 Documentation officielle RxJS - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 Cas d'utilisation typiques

- **Erreur en tant que données** : Traiter les erreurs comme une partie du flux
- **Débogage et journalisation** : Suivi détaillé des notifications
- **Enregistrement des méta-informations** : Enregistrement de la date et du type de notification
- **Combinaison de flux avec erreurs** : Traitement uniforme des erreurs provenant de plusieurs flux

## 🧪 Exemple de code pratique : Erreur comme données

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// Avec materialize, les erreurs deviennent des données
concat(
  of(1, 2),
  throwError(() => new Error('Erreur survenue')),
  of(3, 4)
)
  .pipe(
    materialize(),
    map(notification => {
      if (notification.kind === 'N') {
        return `Valeur : ${notification.value}`;
      } else if (notification.kind === 'E') {
        return `Erreur (données) : ${notification.error?.message}`;
      } else {
        return 'Achèvement';
      }
    })
  )
  .subscribe({
    next: msg => console.log(msg),
    complete: () => console.log('Flux terminé')
  });
// Sortie :
// Valeur : 1
// Valeur : 2
// Erreur (données) : Erreur survenue
// Flux terminé
```

## Manipulation des objets Notification

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // Propriétés de l'objet Notification
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // A une valeur ou non
        value: notification.value,         // Valeur (pour next)
        error: notification.error          // Erreur (pour error)
      };
    })
  )
  .subscribe(console.log);
// Sortie :
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ Notes importantes

### 1. Les erreurs n'interrompent pas le flux

Avec `materialize`, les erreurs sont traitées comme des données et le flux n'est pas interrompu.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('Erreur')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notification :', n.kind),
    error: () => console.log('Gestionnaire d\'erreur'),  // Non appelé
    complete: () => console.log('Complet')
  });
// Sortie :
// Notification : N
// Notification : E  ← L'erreur est traitée comme next
// Complet
```

### 2. Combinaison avec dematerialize

Un flux converti avec `materialize` peut être restauré avec `dematerialize`.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // Traitement intermédiaire ici
    dematerialize()  // Restauration
  )
  .subscribe(console.log);
// Sortie : 1, 2, 3
```

## 📚 Opérateurs associés

- **[dematerialize](./dematerialize)** - Convertir les objets Notification en notifications normales
- **[tap](./tap)** - Exécution d'effets secondaires (pour le débogage)
- **[catchError](/fr/guide/error-handling/retry-catch)** - Gestion des erreurs

## ✅ Résumé

- `materialize` convertit les notifications en objets Notification
- ✅ Les erreurs peuvent être traitées comme des données
- ✅ Utile pour le débogage et la journalisation
- ✅ Permet d'enregistrer les méta-informations des notifications
- ✅ Peut être restauré avec `dematerialize`
- ⚠️ Les erreurs n'interrompent plus le flux
- ⚠️ Attention à l'overhead de performance
