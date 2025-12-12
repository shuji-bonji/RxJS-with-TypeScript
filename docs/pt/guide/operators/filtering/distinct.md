---
description: O operador distinct remove todos os valores duplicados e emite apenas valores únicos que nunca foram emitidos antes. É preciso ter cuidado com streams infinitos, pois ele usa Set internamente para armazenar valores emitidos anteriormente.
---

# distinct - Remover Todos os Valores Duplicados

O operador `distinct` monitora todos os valores emitidos por um Observable e emite **apenas valores que nunca foram emitidos antes**. Internamente, ele usa Set para lembrar valores emitidos anteriormente.


## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5
```

- Remove duplicatas em todo o stream
- Uma vez que um valor é emitido, ele é ignorado não importa quantas vezes apareça posteriormente
- `distinctUntilChanged` remove apenas duplicatas **consecutivas**, enquanto `distinct` remove **todas** as duplicatas

[🌐 Documentação Oficial RxJS - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 Diferença de distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: Remove apenas duplicatas consecutivas
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Saída: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: Remove todas as duplicatas
values$.pipe(
  distinct()
).subscribe(console.log);
// Saída: 1, 2, 3
```

| Operador | Alvo de Remoção | Caso de Uso |
|---|---|---|
| `distinctUntilChanged` | Apenas duplicatas consecutivas | Campos de entrada, dados de sensores |
| `distinct` | Todas as duplicatas | Lista de valores únicos, lista de IDs |


## 🎯 Personalização de Comparação com keySelector

Use a função `keySelector` para determinar duplicatas para uma propriedade específica de um objeto.

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (atualizada)' } as User, // Mesmo ID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // Determinar duplicatas por ID
).subscribe(console.log);
// Saída:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```


## 💡 Padrões de Uso Típicos

1. **Obter Lista de IDs Únicos**
   ```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // Obter apenas IDs de usuário únicos
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`ID do Usuário: ${userId}`);
   });
   // Saída: 1, 2, 3
   ```

2. **Extrair Tipos de Eventos Únicos do Log de Eventos**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // Criar elementos UI dinamicamente
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Botão 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Botão 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = 'Por favor, digite';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // Mesclar múltiplos streams de eventos para extrair tipos de eventos únicos
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'button1-click')),
     fromEvent(button2, 'click').pipe(map(() => 'button2-click')),
     fromEvent(input, 'input').pipe(map(() => 'input-change'))
   );

   events$.pipe(
     distinct(),
     take(3) // Completa quando todos os 3 tipos de eventos estiverem presentes
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Evento único: ${eventType}\n`;
       console.log(`Evento único: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'Todos os tipos de eventos detectados';
     }
   });
   ```


## 🧠 Exemplo de Código Prático (Entrada de Tags)

Aqui está um exemplo de uma UI que remove automaticamente duplicatas de tags inseridas pelo usuário.

```ts
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// Criar elementos UI
const container = document.createElement('div');
document.body.appendChild(container);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'Digite uma tag e pressione Enter';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// Stream de adição de tag
const tagSubject$ = new Subject<string>();

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase()),
  distinct() // Remover tags duplicadas
).subscribe(tag => {
  const li = document.createElement('li');
  li.textContent = tag;
  tagList.appendChild(li);
});

// Adicionar uma tag com a tecla Enter
fromEvent<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (value) {
      tagSubject$.next(value);
      tagInput.value = '';
    }
  }
});
```

Este código garante que a mesma tag seja adicionada à lista apenas uma vez, mesmo que seja inserida várias vezes.


## ⚠️ Observação sobre Uso de Memória

> [!WARNING]
> O operador `distinct` usa **Set** internamente para armazenar todos os valores emitidos anteriormente. Usá-lo com um stream infinito pode causar vazamentos de memória.

### Problema: Vazamento de Memória em Streams Infinitos

```ts
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ Exemplo ruim: Usando distinct com streams infinitos
interval(100).pipe(
  map(n => n % 10), // Ciclo 0-9
  distinct() // Emite apenas os primeiros 10, depois mantém na memória
).subscribe(console.log);
// Saída: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// Nada é emitido depois disso, mas o Set continua sendo armazenado
```

### Solução: Limpar Set com Parâmetro flushes

```ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ Bom exemplo: Limpar Set periodicamente
interval(100).pipe(
  map(n => n % 5),
  distinct(
    value => value,
    timer(1000) // Limpar Set a cada 1 segundo
  )
).subscribe(console.log);
// A cada 1 segundo, 0, 1, 2, 3, 4 são emitidos novamente
```

### Melhores Práticas

1. **Use com streams finitos**: Respostas HTTP, conversão de arrays, etc.
2. **Use flushes**: Limpe periodicamente para streams infinitos
3. **Considere distinctUntilChanged**: Use isto para remover apenas duplicatas consecutivas


## 📋 Uso Type-Safe

Aqui está um exemplo de uma implementação type-safe utilizando generics em TypeScript.

```ts
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interface Product {
  id: number;
  name: string;
  categoryId: number;
}

function getUniqueCategories(
  products$: Observable<Product>
): Observable<number> {
  return products$.pipe(
    distinct(product => product.categoryId)
  ).pipe(
    map(product => product.categoryId)
  );
}

// Exemplo de uso
import { of } from 'rxjs';

const products$ = of(
  { id: 1, name: 'Laptop', categoryId: 10 } as Product,
  { id: 2, name: 'Mouse', categoryId: 10 } as Product,
  { id: 3, name: 'Livro', categoryId: 20 } as Product
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`ID da Categoria: ${categoryId}`);
});
// Saída: 10, 20
```


## 🎓 Resumo

### Quando Usar distinct
- ✅ Quando você precisa de uma lista de valores únicos
- ✅ Quando você quer remover duplicatas em um stream finito
- ✅ Criando uma lista de IDs ou categorias

### Quando Usar distinctUntilChanged
- ✅ Quando você quer remover apenas duplicatas consecutivas
- ✅ Detecção de mudança em campo de entrada
- ✅ Quando você quer economizar memória com streams infinitos

### Observações
- ⚠️ Use o parâmetro `flushes` para streams infinitos para evitar vazamentos de memória
- ⚠️ Esteja ciente do uso de memória quando grandes números de valores únicos são transmitidos
- ⚠️ Se o desempenho for crítico, monitore o tamanho do Set


## 🚀 Próximos Passos

- **[distinctUntilChanged](/pt/guide/operators/filtering/distinctUntilChanged)** - Aprenda como remover apenas duplicatas consecutivas
- **[distinctUntilKeyChanged](/pt/guide/operators/filtering/distinctUntilKeyChanged)** - Aprenda como comparar objetos por chave
- **[filter](/pt/guide/operators/filtering/filter)** - Aprenda como filtrar com base em condições
- **[Exemplos Práticos de Operadores de Filtragem](/pt/guide/operators/filtering/practical-use-cases)** - Aprenda casos de uso reais
